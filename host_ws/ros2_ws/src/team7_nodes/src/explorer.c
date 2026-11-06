#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <action_msgs/msg/goal_status.h>
#include <geometry_msgs/msg/point.h>
#include <nav2_msgs/action/navigate_to_pose.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <rcl_action/rcl_action.h>
#include <rclc/action_client.h>
#include <std_msgs/msg/string.h>
#include <visualization_msgs/msg/marker_array.h>

#include "common.h"
#include "tb4/explore.h"

#define LOG "explorer"
#define MAX_BLACKLIST 256
#define MAX_PENDING 8

typedef nav2_msgs__action__NavigateToPose_SendGoal_Request goal_req_t;
typedef nav2_msgs__action__NavigateToPose_GetResult_Response result_t;
typedef nav2_msgs__action__NavigateToPose_FeedbackMessage feedback_t;

typedef struct {
  bool used;
  uint8_t uuid[16];
  double gx, gy;
} pending_t;

typedef struct {
  bool active;
  uint8_t uuid[16];
  rclc_action_goal_handle_t *handle;
  double gx, gy;
  double started, last_progress, best_dist;
} active_t;

typedef struct {
  char map_frame[64], base_frame[64];
  double goal_timeout, progress_timeout, progress_epsilon, gone_radius;
  tb4_explore_params ep;
  tb4_grid grid;
  bool have_grid;
  t7_tf_listener tf;
  volatile bool done;
  tb4_frontier_set fs;
  double blacklist[MAX_BLACKLIST][2];
  size_t n_blacklist;
  active_t act;
  pending_t pending[MAX_PENDING];
  bool finished_exploring, nav_ready;
  rcl_node_t *node;
  rclc_action_client_t client;
  goal_req_t req;
  rcl_publisher_t pub_frontiers;
  visualization_msgs__msg__MarkerArray viz;
} state_t;

static state_t S;

static double now_s(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void add_blacklist(double x, double y)
{
  if (S.n_blacklist < MAX_BLACKLIST) {
    S.blacklist[S.n_blacklist][0] = x;
    S.blacklist[S.n_blacklist][1] = y;
    S.n_blacklist++;
  }
}

static pending_t *find_pending(const uint8_t *uuid)
{
  for (int i = 0; i < MAX_PENDING; i++) {
    if (S.pending[i].used && memcmp(S.pending[i].uuid, uuid, 16) == 0) return &S.pending[i];
  }
  return NULL;
}

static void finish_goal(const uint8_t *uuid, int8_t status, const char *why)
{
  pending_t *p = find_pending(uuid);
  if (!p) return;
  bool ok = status == action_msgs__msg__GoalStatus__STATUS_SUCCEEDED ||
            status == action_msgs__msg__GoalStatus__STATUS_CANCELED;
  if (!ok) {
    RCUTILS_LOG_WARN_NAMED(LOG, "goal (%.2f, %.2f) %s (status %d), blacklisting", p->gx, p->gy, why, status);
    add_blacklist(p->gx, p->gy);
  }
  if (S.act.active && memcmp(S.act.uuid, uuid, 16) == 0) {
    S.act.active = false;
    S.act.handle = NULL;
  }
  p->used = false;
}

static void goal_cb(rclc_action_goal_handle_t *h, bool accepted, void *ctx)
{
  (void)ctx;
  if (!accepted) {
    finish_goal(h->goal_id.uuid, action_msgs__msg__GoalStatus__STATUS_ABORTED, "rejected");
  }
}

static void feedback_cb(rclc_action_goal_handle_t *h, void *fb, void *ctx)
{
  (void)h; (void)fb; (void)ctx;
}

static void result_cb(rclc_action_goal_handle_t *h, void *res, void *ctx)
{
  (void)ctx;
  finish_goal(h->goal_id.uuid, ((result_t *)res)->status, "ended");
}

static void cancel_cb(rclc_action_goal_handle_t *h, bool cancelled, void *ctx)
{
  (void)h; (void)cancelled; (void)ctx;
}

static void cancel_active(void)
{
  if (S.act.active && S.act.handle) {
    T7_IGNORE(rclc_action_send_cancel_request(S.act.handle));
  }
  S.act.active = false;
  S.act.handle = NULL;
}

static void costmap_cb(const void *msg, void *ctx)
{
  (void)ctx;
  S.have_grid = t7_grid_from_msg(&S.grid, msg);
}

static void mission_cb(const void *msg, void *ctx)
{
  (void)ctx;
  const std_msgs__msg__String *m = msg;
  if (m->data.data && strcmp(m->data.data, "DONE") == 0) S.done = true;
}

static void publish_frontiers(void)
{
  visualization_msgs__msg__Marker__Sequence *seq = &S.viz.markers;
  visualization_msgs__msg__Marker__Sequence__fini(seq);
  if (!visualization_msgs__msg__Marker__Sequence__init(seq, 2)) return;
  t7_marker_setup(&seq->data[0], S.map_frame, "", 0, 0, T7_MARKER_DELETEALL);
  visualization_msgs__msg__Marker *m = &seq->data[1];
  t7_marker_setup(m, S.map_frame, "frontier_goals", 0, T7_MARKER_POINTS, T7_MARKER_ADD);
  m->scale.x = m->scale.y = 0.12;
  t7_marker_color(m, 0.2f, 0.6f, 1.0f, 0.9f);
  if (geometry_msgs__msg__Point__Sequence__init(&m->points, S.fs.len)) {
    for (size_t i = 0; i < S.fs.len; i++) {
      m->points.data[i].x = S.fs.items[i].gx;
      m->points.data[i].y = S.fs.items[i].gy;
      m->points.data[i].z = 0.05;
    }
  }
  T7_IGNORE(rcl_publish(&S.pub_frontiers, &S.viz, NULL));
}

static void send_goal(const tb4_frontier *f, tb4_iso robot)
{
  pending_t *slot = NULL;
  for (int i = 0; i < MAX_PENDING && !slot; i++) {
    if (!S.pending[i].used) slot = &S.pending[i];
  }
  if (!slot) {
    RCUTILS_LOG_WARN_NAMED(LOG, "too many pending goals");
    return;
  }
  double yaw = atan2(f->gy - robot.t.y, f->gx - robot.t.x);
  tb4_quat q = tb4_quat_from_yaw(yaw);
  geometry_msgs__msg__PoseStamped *ps = &S.req.goal.pose;
  rosidl_runtime_c__String__assign(&ps->header.frame_id, S.map_frame);
  ps->pose.position.x = f->gx;
  ps->pose.position.y = f->gy;
  ps->pose.position.z = 0.0;
  ps->pose.orientation.x = q.x;
  ps->pose.orientation.y = q.y;
  ps->pose.orientation.z = q.z;
  ps->pose.orientation.w = q.w;

  rclc_action_goal_handle_t *h = NULL;
  if (rclc_action_send_goal_request(&S.client, &S.req, &h) != RCL_RET_OK || !h) {
    RCUTILS_LOG_WARN_NAMED(LOG, "send_goal failed: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return;
  }
  RCUTILS_LOG_INFO_NAMED(LOG, "new goal (%.2f, %.2f) frontier %.1f m, %.1f m away", f->gx, f->gy,
                         f->size_m, f->distance);
  slot->used = true;
  memcpy(slot->uuid, h->goal_id.uuid, 16);
  slot->gx = f->gx;
  slot->gy = f->gy;
  double t = now_s();
  S.act = (active_t){true, {0}, h, f->gx, f->gy, t, t, f->distance};
  memcpy(S.act.uuid, h->goal_id.uuid, 16);
}

static void tick(rcl_timer_t *timer, int64_t last_call)
{
  (void)timer; (void)last_call;
  if (!S.nav_ready) {
    bool avail = false;
    if (rcl_action_server_is_available(S.node, &S.client.rcl_handle, &avail) == RCL_RET_OK && avail) {
      S.nav_ready = true;
      RCUTILS_LOG_INFO_NAMED(LOG, "Nav2 is up, exploring");
    } else {
      rcl_reset_error();
      static unsigned waits;
      if (waits++ % 5 == 0) {
        RCUTILS_LOG_INFO_NAMED(LOG, "waiting for Nav2 action server");
      }
      return;
    }
  }
  if (S.done) {
    if (S.act.active) {
      RCUTILS_LOG_INFO_NAMED(LOG, "mission done, cancelling navigation");
      cancel_active();
    }
    return;
  }
  if (!S.have_grid) {
    RCUTILS_LOG_WARN_NAMED(LOG, "waiting for costmap");
    return;
  }
  tb4_iso robot;
  tb4_tf_status st = tb4_tf_lookup(S.tf.buf, S.map_frame, S.base_frame, TB4_TF_LATEST, &robot);
  if (st != TB4_TF_OK) {
    RCUTILS_LOG_WARN_NAMED(LOG, "no robot pose yet: %s", tb4_tf_status_str(st));
    return;
  }
  if (tb4_find_frontiers(&S.grid, robot.t.x, robot.t.y, &S.ep, &S.fs) < 0) {
    RCUTILS_LOG_ERROR_NAMED(LOG, "out of memory in frontier search");
    return;
  }
  publish_frontiers();

  double t = now_s();
  if (S.act.active) {
    double d = hypot(robot.t.x - S.act.gx, robot.t.y - S.act.gy);
    if (d < S.act.best_dist - S.progress_epsilon) {
      S.act.best_dist = d;
      S.act.last_progress = t;
    }
    bool gone = true;
    for (size_t i = 0; i < S.fs.len && gone; i++) {
      gone = hypot(S.fs.items[i].gx - S.act.gx, S.fs.items[i].gy - S.act.gy) >= S.gone_radius;
    }
    const char *why = NULL;
    bool blacklist = true;
    if (t - S.act.started > S.goal_timeout) why = "timeout";
    else if (t - S.act.last_progress > S.progress_timeout) why = "no progress";
    else if (gone) { why = "frontier explored"; blacklist = false; }
    if (why) {
      RCUTILS_LOG_INFO_NAMED(LOG, "dropping goal (%.2f, %.2f): %s", S.act.gx, S.act.gy, why);
      if (blacklist) add_blacklist(S.act.gx, S.act.gy);
      cancel_active();
    }
  }
  if (S.act.active) return;

  const tb4_frontier *f = tb4_select_goal(&S.fs, (const double(*)[2])S.blacklist, S.n_blacklist, &S.ep);
  if (!f) {
    if (!S.finished_exploring) {
      RCUTILS_LOG_INFO_NAMED(LOG, "no reachable frontiers left: exploration finished");
      S.finished_exploring = true;
    }
    return;
  }
  S.finished_exploring = false;
  send_goal(f, robot);
}

int main(int argc, char **argv)
{
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rclc_support_t support;
  T7_CHECK(rclc_support_init(&support, argc, (const char *const *)argv, &alloc));
  rcl_node_t node = rcl_get_zero_initialized_node();
  T7_CHECK(rclc_node_init_default(&node, "explorer", "", &support));
  S.node = &node;
  t7_install_signal_handlers();

  t7_params p;
  t7_params_init(&p, &support, &node);
  char costmap_topic[256], nav_action[256], tf_topic[256], tfs_topic[256];
  t7_param_string(&p, "map_frame", "map", S.map_frame, sizeof(S.map_frame));
  t7_param_string(&p, "base_frame", "base_link", S.base_frame, sizeof(S.base_frame));
  t7_param_string(&p, "costmap_topic", "/global_costmap/costmap", costmap_topic, sizeof(costmap_topic));
  t7_param_string(&p, "nav_action", "/navigate_to_pose", nav_action, sizeof(nav_action));
  t7_param_string(&p, "tf_topic", "/tf", tf_topic, sizeof(tf_topic));
  t7_param_string(&p, "tf_static_topic", "/tf_static", tfs_topic, sizeof(tfs_topic));
  double period = t7_param_double(&p, "period", 1.0);
  S.goal_timeout = t7_param_double(&p, "goal_timeout", 60.0);
  S.progress_timeout = t7_param_double(&p, "progress_timeout", 15.0);
  S.progress_epsilon = t7_param_double(&p, "progress_epsilon", 0.1);
  S.gone_radius = t7_param_double(&p, "frontier_gone_radius", 0.5);
  S.ep = tb4_explore_default_params();
  S.ep.free_threshold = (int8_t)t7_param_int(&p, "free_threshold", S.ep.free_threshold);
  S.ep.min_frontier_cells = (size_t)t7_param_int(&p, "min_frontier_cells", (int64_t)S.ep.min_frontier_cells);
  S.ep.size_weight = t7_param_double(&p, "size_weight", S.ep.size_weight);
  S.ep.distance_weight = t7_param_double(&p, "distance_weight", S.ep.distance_weight);
  S.ep.blacklist_radius = t7_param_double(&p, "blacklist_radius", S.ep.blacklist_radius);
  S.ep.min_goal_distance = t7_param_double(&p, "min_goal_distance", S.ep.min_goal_distance);
  t7_params_fini(&p);

  tb4_grid_init_empty(&S.grid);
  tb4_frontier_set_init(&S.fs);
  visualization_msgs__msg__MarkerArray__init(&S.viz);
  nav2_msgs__action__NavigateToPose_SendGoal_Request__init(&S.req);

  rclc_executor_t ex = rclc_executor_get_zero_initialized_executor();
  T7_CHECK(rclc_executor_init(&ex, &support.context, 6, &alloc));
  t7_tf_listener_init(&S.tf, &node, &ex, tf_topic, tfs_topic, 10.0, 0.2);

  rcl_subscription_t sub_cm = rcl_get_zero_initialized_subscription();
  rmw_qos_profile_t latched = t7_qos(1, true, true);
  T7_CHECK(rclc_subscription_init(&sub_cm, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, OccupancyGrid),
                                  costmap_topic, &latched));
  nav_msgs__msg__OccupancyGrid cm_msg;
  nav_msgs__msg__OccupancyGrid__init(&cm_msg);
  T7_CHECK(rclc_executor_add_subscription_with_context(&ex, &sub_cm, &cm_msg, costmap_cb, NULL, ON_NEW_DATA));

  rcl_subscription_t sub_ms = rcl_get_zero_initialized_subscription();
  T7_CHECK(rclc_subscription_init(&sub_ms, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                  "/team7/mission_state", &latched));
  std_msgs__msg__String ms_msg;
  std_msgs__msg__String__init(&ms_msg);
  T7_CHECK(rclc_executor_add_subscription_with_context(&ex, &sub_ms, &ms_msg, mission_cb, NULL, ON_NEW_DATA));

  T7_CHECK(rclc_action_client_init_default(&S.client, &node,
                                           ROSIDL_GET_ACTION_TYPE_SUPPORT(nav2_msgs, NavigateToPose), nav_action));
  static result_t result;
  static feedback_t feedback;
  nav2_msgs__action__NavigateToPose_GetResult_Response__init(&result);
  nav2_msgs__action__NavigateToPose_FeedbackMessage__init(&feedback);
  T7_CHECK(rclc_executor_add_action_client(&ex, &S.client, MAX_PENDING, &result, &feedback, goal_cb,
                                           feedback_cb, result_cb, cancel_cb, NULL));

  S.pub_frontiers = rcl_get_zero_initialized_publisher();
  T7_CHECK(rclc_publisher_init_default(&S.pub_frontiers, &node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, MarkerArray),
                                       "/team7/frontiers"));
  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  const uint64_t period_ns = period > 0.0 ? (uint64_t)(period * 1e9) : RCL_S_TO_NS(1);
  T7_CHECK(rclc_timer_init_default2(&timer, &support, period_ns, tick, true));
  T7_CHECK(rclc_executor_add_timer(&ex, &timer));
  RCUTILS_LOG_INFO_NAMED(LOG, "waiting for %s ...", nav_action);

  t7_spin(&ex, NULL);

  cancel_active();
  T7_IGNORE(rclc_executor_spin_some(&ex, RCL_MS_TO_NS(50)));
  T7_IGNORE(rclc_executor_fini(&ex));
  T7_IGNORE(rcl_timer_fini(&timer));
  T7_IGNORE(rcl_publisher_fini(&S.pub_frontiers, &node));
  T7_IGNORE(rclc_action_client_fini(&S.client, &node));
  T7_IGNORE(rcl_subscription_fini(&sub_ms, &node));
  T7_IGNORE(rcl_subscription_fini(&sub_cm, &node));
  t7_tf_listener_fini(&S.tf, &node);
  nav2_msgs__action__NavigateToPose_SendGoal_Request__fini(&S.req);
  nav2_msgs__action__NavigateToPose_GetResult_Response__fini(&result);
  nav2_msgs__action__NavigateToPose_FeedbackMessage__fini(&feedback);
  nav_msgs__msg__OccupancyGrid__fini(&cm_msg);
  std_msgs__msg__String__fini(&ms_msg);
  visualization_msgs__msg__MarkerArray__fini(&S.viz);
  tb4_frontier_set_free(&S.fs);
  tb4_grid_free(&S.grid);
  T7_IGNORE(rcl_node_fini(&node));
  T7_IGNORE(rclc_support_fini(&support));
  return 0;
}
