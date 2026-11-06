#include <errno.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <action_msgs/srv/cancel_goal.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <visualization_msgs/msg/marker_array.h>

#include "common.h"

#define LOG "mission"
#define MAX_IDS 64

extern char **environ;

typedef struct {
  size_t expected;
  double time_limit, autosave_period;
  char out_dir[2048], map_name[128], map_topic[256];
  int32_t found[MAX_IDS];
  size_t n_found, last_count;
  double start, last_save;
  pid_t saver;
  volatile bool finished;
  rcl_publisher_t pub_state, pub_resume;
  rcl_client_t cancel;
} state_t;

static state_t S;

static double now_s(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void publish_state(const char *s)
{
  std_msgs__msg__String m;
  std_msgs__msg__String__init(&m);
  rosidl_runtime_c__String__assign(&m.data, s);
  T7_IGNORE(rcl_publish(&S.pub_state, &m, NULL));
  std_msgs__msg__String__fini(&m);
}

static void targets_cb(const void *msg, void *ctx)
{
  (void)ctx;
  const visualization_msgs__msg__MarkerArray *a = msg;
  for (size_t i = 0; i < a->markers.size; i++) {
    const visualization_msgs__msg__Marker *m = &a->markers.data[i];
    if (!m->ns.data || strcmp(m->ns.data, "target_confirmed") != 0) continue;
    bool seen = false;
    for (size_t k = 0; k < S.n_found && !seen; k++) seen = S.found[k] == m->id;
    if (!seen && S.n_found < MAX_IDS) S.found[S.n_found++] = m->id;
  }
}

static pid_t spawn_map_saver(const char *name)
{
  char path[4096];
  snprintf(path, sizeof(path), "%s/%s", S.out_dir, name);
  char *argv[] = {"ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path, "-t", S.map_topic,
                  "--ros-args", "-p", "save_map_timeout:=10.0", "-p", "map_subscribe_transient_local:=true",
                  "--log-level", "warn", NULL};
  pid_t pid;
  int rc = posix_spawnp(&pid, "ros2", NULL, NULL, argv, environ);
  if (rc != 0) {
    RCUTILS_LOG_ERROR_NAMED(LOG, "posix_spawnp(ros2) failed: %s", strerror(rc));
    return -1;
  }
  return pid;
}

static void finish(const char *why)
{
  double elapsed = now_s() - S.start;
  RCUTILS_LOG_INFO_NAMED(LOG, "DONE (%s) after %.1f s with %zu/%zu targets", why, elapsed, S.n_found, S.expected);
  publish_state("DONE");
  std_msgs__msg__Bool stop = {.data = false};
  T7_IGNORE(rcl_publish(&S.pub_resume, &stop, NULL));

  action_msgs__srv__CancelGoal_Request req;
  action_msgs__srv__CancelGoal_Request__init(&req);
  int64_t seq;
  /* empty goal_info cancels all goals. no wait for reply, it hangs if nav2 is down */
  if (rcl_send_request(&S.cancel, &req, &seq) != RCL_RET_OK) {
    RCUTILS_LOG_WARN_NAMED(LOG, "cancel_goal request failed: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
  action_msgs__srv__CancelGoal_Request__fini(&req);

  if (S.saver > 0) {
    (void)waitpid(S.saver, NULL, 0);
    S.saver = 0;
  }
  pid_t pid = spawn_map_saver(S.map_name);
  int status = 0;
  if (pid > 0 && waitpid(pid, &status, 0) == pid && WIFEXITED(status) && WEXITSTATUS(status) == 0) {
    RCUTILS_LOG_INFO_NAMED(LOG, "map saved to %s/%s", S.out_dir, S.map_name);
  } else {
    RCUTILS_LOG_ERROR_NAMED(LOG, "map_saver_cli failed (status %d)", status);
  }
  RCUTILS_LOG_INFO_NAMED(LOG, "targets: %s/targets.csv", S.out_dir);
  S.finished = true;
}

static void tick(rcl_timer_t *timer, int64_t last_call)
{
  (void)timer; (void)last_call;
  if (S.finished) return;
  double elapsed = now_s() - S.start;
  if (S.n_found != S.last_count) {
    char ids[512] = "";
    for (size_t i = 0; i < S.n_found; i++) {
      size_t l = strlen(ids);
      snprintf(ids + l, sizeof(ids) - l, "%s%d", i ? ", " : "", S.found[i]);
    }
    RCUTILS_LOG_INFO_NAMED(LOG, "%zu/%zu targets confirmed at %.1f s: {%s}", S.n_found, S.expected, elapsed, ids);
    S.last_count = S.n_found;
  }
  if (S.saver > 0 && waitpid(S.saver, NULL, WNOHANG) == S.saver) {
    S.saver = 0;
  }
  if (S.n_found >= S.expected) {
    finish("all targets found");
    return;
  }
  if (elapsed >= S.time_limit) {
    finish("time limit");
    return;
  }
  if (S.saver == 0 && now_s() - S.last_save >= S.autosave_period) {
    S.last_save = now_s();
    char name[160];
    snprintf(name, sizeof(name), "%s_autosave", S.map_name);
    pid_t pid = spawn_map_saver(name);
    S.saver = pid > 0 ? pid : 0;
  }
}

int main(int argc, char **argv)
{
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rclc_support_t support;
  T7_CHECK(rclc_support_init(&support, argc, (const char *const *)argv, &alloc));
  rcl_node_t node = rcl_get_zero_initialized_node();
  T7_CHECK(rclc_node_init_default(&node, "mission", "", &support));
  t7_install_signal_handlers();

  t7_params p;
  t7_params_init(&p, &support, &node);
  char out_raw[1024], nav_action[256];
  S.expected = (size_t)t7_param_int(&p, "expected_targets", 3);
  S.time_limit = t7_param_double(&p, "time_limit", 590.0);
  S.autosave_period = t7_param_double(&p, "autosave_period", 60.0);
  t7_param_string(&p, "output_dir", "~/team7_run", out_raw, sizeof(out_raw));
  t7_param_string(&p, "map_name", "map", S.map_name, sizeof(S.map_name));
  t7_param_string(&p, "map_topic", "/map", S.map_topic, sizeof(S.map_topic));
  t7_param_string(&p, "nav_action", "/navigate_to_pose", nav_action, sizeof(nav_action));
  t7_params_fini(&p);
  t7_expand_home(out_raw, S.out_dir, sizeof(S.out_dir));
  if (t7_mkdirs(S.out_dir) != 0) {
    RCUTILS_LOG_FATAL_NAMED(LOG, "cannot create %s: %s", S.out_dir, strerror(errno));
    return 1;
  }

  rmw_qos_profile_t latched = t7_qos(1, true, true);
  S.pub_state = rcl_get_zero_initialized_publisher();
  T7_CHECK(rclc_publisher_init(&S.pub_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                               "/team7/mission_state", &latched));
  S.pub_resume = rcl_get_zero_initialized_publisher();
  T7_CHECK(rclc_publisher_init_default(&S.pub_resume, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                       "/explore/resume"));
  char cancel_srv[300];
  snprintf(cancel_srv, sizeof(cancel_srv), "%s/_action/cancel_goal", nav_action);
  S.cancel = rcl_get_zero_initialized_client();
  T7_CHECK(rclc_client_init_default(&S.cancel, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(action_msgs, srv, CancelGoal),
                                    cancel_srv));

  rclc_executor_t ex = rclc_executor_get_zero_initialized_executor();
  T7_CHECK(rclc_executor_init(&ex, &support.context, 2, &alloc));
  rcl_subscription_t sub = rcl_get_zero_initialized_subscription();
  T7_CHECK(rclc_subscription_init_default(&sub, &node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, MarkerArray),
                                          "/team7/targets"));
  visualization_msgs__msg__MarkerArray targets;
  visualization_msgs__msg__MarkerArray__init(&targets);
  T7_CHECK(rclc_executor_add_subscription_with_context(&ex, &sub, &targets, targets_cb, NULL, ON_NEW_DATA));
  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  T7_CHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(500), tick, true));
  T7_CHECK(rclc_executor_add_timer(&ex, &timer));

  S.start = S.last_save = now_s();
  publish_state("RUNNING");
  RCUTILS_LOG_INFO_NAMED(LOG, "mission started: %zu targets, %.0f s limit, output %s", S.expected,
                         S.time_limit, S.out_dir);

  t7_spin(&ex, NULL);

  if (S.saver > 0) {
    (void)waitpid(S.saver, NULL, 0);
  }
  T7_IGNORE(rclc_executor_fini(&ex));
  T7_IGNORE(rcl_timer_fini(&timer));
  T7_IGNORE(rcl_subscription_fini(&sub, &node));
  T7_IGNORE(rcl_client_fini(&S.cancel, &node));
  T7_IGNORE(rcl_publisher_fini(&S.pub_resume, &node));
  T7_IGNORE(rcl_publisher_fini(&S.pub_state, &node));
  visualization_msgs__msg__MarkerArray__fini(&targets);
  T7_IGNORE(rcl_node_fini(&node));
  T7_IGNORE(rclc_support_fini(&support));
  return 0;
}
