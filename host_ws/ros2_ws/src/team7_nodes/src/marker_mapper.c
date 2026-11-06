#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <aruco_opencv_msgs/msg/aruco_detection.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <visualization_msgs/msg/marker_array.h>

#include "common.h"
#include "tb4/markers.h"

#define LOG "marker_mapper"
#define MAX_IGNORE 32

typedef struct {
  tb4_grid grid;
  bool have_grid;
  tb4_tracker tracker;
  tb4_locate_params locate;
  t7_tf_listener tf;
  char map_frame[64];
  int64_t ignore[MAX_IGNORE];
  size_t n_ignore;
  char csv_path[4096];
  char last_csv[8192];
  uint64_t tf_failures;
  uint64_t stats[2];
  rcl_publisher_t pub_targets;
  visualization_msgs__msg__MarkerArray out;
} state_t;

static bool ignored(const state_t *s, int64_t id)
{
  for (size_t i = 0; i < s->n_ignore; i++) {
    if (s->ignore[i] == id) return true;
  }
  return false;
}

static void map_cb(const void *msg, void *ctx)
{
  state_t *s = ctx;
  s->have_grid = t7_grid_from_msg(&s->grid, msg);
}

static void publish_markers(state_t *s, const tb4_target *t, size_t n)
{
  visualization_msgs__msg__Marker__Sequence *seq = &s->out.markers;
  visualization_msgs__msg__Marker__Sequence__fini(seq);
  if (!visualization_msgs__msg__Marker__Sequence__init(seq, 1 + 2 * n)) {
    return;
  }
  t7_marker_setup(&seq->data[0], s->map_frame, "", 0, 0, T7_MARKER_DELETEALL);
  for (size_t i = 0; i < n; i++) {
    const bool c = t[i].confirmed;
    char ns[64], label[96];
    visualization_msgs__msg__Marker *sp = &seq->data[1 + 2 * i];
    visualization_msgs__msg__Marker *tx = &seq->data[2 + 2 * i];
    t7_marker_setup(sp, s->map_frame, c ? "target_confirmed" : "target_tentative", (int32_t)t[i].id,
                    T7_MARKER_SPHERE, T7_MARKER_ADD);
    sp->pose.position.x = t[i].x;
    sp->pose.position.y = t[i].y;
    sp->pose.position.z = 0.15;
    sp->scale.x = sp->scale.y = sp->scale.z = 0.2;
    if (c) t7_marker_color(sp, 0.1f, 0.9f, 0.2f, 0.9f);
    else t7_marker_color(sp, 1.0f, 0.8f, 0.0f, 0.6f);

    snprintf(ns, sizeof(ns), "%s_label", c ? "target_confirmed" : "target_tentative");
    t7_marker_setup(tx, s->map_frame, ns, (int32_t)t[i].id, T7_MARKER_TEXT, T7_MARKER_ADD);
    tx->pose.position.x = t[i].x;
    tx->pose.position.y = t[i].y;
    tx->pose.position.z = 0.45;
    tx->scale.z = 0.25;
    t7_marker_color(tx, 1, 1, 1, 1);
    if (c) snprintf(label, sizeof(label), "ID %" PRId64 " (%.2f, %.2f)", t[i].id, t[i].x, t[i].y);
    else snprintf(label, sizeof(label), "ID %" PRId64 "? %zuv", t[i].id, t[i].views);
    rosidl_runtime_c__String__assign(&tx->text, label);
  }
  T7_IGNORE(rcl_publish(&s->pub_targets, &s->out, NULL));
}

static void detection_cb(const void *msg, void *ctx)
{
  state_t *s = ctx;
  const aruco_opencv_msgs__msg__ArucoDetection *det = msg;
  if (det->markers.size == 0) {
    return;
  }
  tb4_iso cam;
  tb4_tf_status st = tb4_tf_lookup(s->tf.buf, s->map_frame, det->header.frame_id.data,
                                   t7_stamp(&det->header.stamp), &cam);
  if (st != TB4_TF_OK) {
    if (s->tf_failures++ % 50 == 0) {
      RCUTILS_LOG_WARN_NAMED(LOG, "tf %s <- %s: %s (%" PRIu64 " failures)", s->map_frame,
                             det->header.frame_id.data, tb4_tf_status_str(st), s->tf_failures);
    }
    return;
  }

  tb4_target before[TB4_MAX_TRACKED_IDS];
  size_t nb = tb4_tracker_targets(&s->tracker, before, TB4_MAX_TRACKED_IDS);

  for (size_t i = 0; i < det->markers.size; i++) {
    const aruco_opencv_msgs__msg__MarkerPose *m = &det->markers.data[i];
    int64_t id = m->marker_id;
    if (ignored(s, id)) continue;
    tb4_vec3 t = {m->pose.position.x, m->pose.position.y, m->pose.position.z};
    double range = tb4_vec3_norm(t);
    if (range < 1e-6) continue;
    tb4_vec3 unit = {t.x / range, t.y / range, t.z / range};
    tb4_vec3 b = tb4_quat_rotate(cam.q, unit);
    tb4_observation o = {id, {cam.t.x, cam.t.y, cam.t.z}, {b.x, b.y, b.z}, range};
    tb4_estimate e;
    if (tb4_locate(s->have_grid ? &s->grid : NULL, &o, &s->locate, &e)) {
      s->stats[e.source]++;
      if (tb4_tracker_add(&s->tracker, id, &e) != 0) {
        RCUTILS_LOG_WARN_NAMED(LOG, "tracker full, dropping id %" PRId64, id);
      }
    }
  }

  tb4_target all[TB4_MAX_TRACKED_IDS], conf[TB4_MAX_TRACKED_IDS];
  size_t n = tb4_tracker_targets(&s->tracker, all, TB4_MAX_TRACKED_IDS), nc = 0;
  for (size_t i = 0; i < n; i++) {
    if (!all[i].confirmed) continue;
    conf[nc++] = all[i];
    bool was = false;
    for (size_t k = 0; k < nb; k++) {
      if (before[k].id == all[i].id && before[k].confirmed) was = true;
    }
    if (!was) {
      RCUTILS_LOG_INFO_NAMED(LOG, "CONFIRMED target id=%" PRId64 " at (%.2f, %.2f) from %zu views",
                             all[i].id, all[i].x, all[i].y, all[i].views);
    }
  }
  char csv[sizeof(s->last_csv)];
  int len = tb4_targets_csv(conf, nc, csv, sizeof(csv));
  if (len > 0 && (size_t)len < sizeof(csv) && strcmp(csv, s->last_csv) != 0) {
    if (t7_write_atomic(s->csv_path, csv, (size_t)len) == 0) {
      memcpy(s->last_csv, csv, (size_t)len + 1);
    } else {
      RCUTILS_LOG_WARN_NAMED(LOG, "cannot write %s", s->csv_path);
    }
  }
  publish_markers(s, all, n);
}

int main(int argc, char **argv)
{
  static state_t s;
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rclc_support_t support;
  T7_CHECK(rclc_support_init(&support, argc, (const char *const *)argv, &alloc));
  rcl_node_t node = rcl_get_zero_initialized_node();
  T7_CHECK(rclc_node_init_default(&node, "marker_mapper", "", &support));
  t7_install_signal_handlers();

  t7_params p;
  t7_params_init(&p, &support, &node);
  char det_topic[256], map_topic[256], tf_topic[256], tfs_topic[256], out_raw[1024], out_dir[2048], csv[256];
  t7_param_string(&p, "detections_topic", "/aruco_detections", det_topic, sizeof(det_topic));
  t7_param_string(&p, "map_topic", "/map", map_topic, sizeof(map_topic));
  t7_param_string(&p, "tf_topic", "/tf", tf_topic, sizeof(tf_topic));
  t7_param_string(&p, "tf_static_topic", "/tf_static", tfs_topic, sizeof(tfs_topic));
  t7_param_string(&p, "map_frame", "map", s.map_frame, sizeof(s.map_frame));
  t7_param_string(&p, "output_dir", "~/team7_run", out_raw, sizeof(out_raw));
  t7_param_string(&p, "csv_name", "targets.csv", csv, sizeof(csv));
  s.locate = tb4_locate_default_params();
  s.locate.max_range = t7_param_double(&p, "max_range", s.locate.max_range);
  s.locate.occupied_threshold = (int8_t)t7_param_int(&p, "occupied_threshold", s.locate.occupied_threshold);
  s.locate.range_ratio_gate = t7_param_double(&p, "range_ratio_gate", s.locate.range_ratio_gate);
  s.locate.allow_pnp_fallback = t7_param_bool(&p, "allow_pnp_fallback", s.locate.allow_pnp_fallback);
  s.locate.max_abs_bearing_z = t7_param_double(&p, "max_abs_bearing_z", s.locate.max_abs_bearing_z);
  tb4_track_params tp = tb4_track_default_params();
  tp.min_views = (size_t)t7_param_int(&p, "min_views", (int64_t)tp.min_views);
  tp.max_spread = t7_param_double(&p, "max_spread", tp.max_spread);
  tp.max_history = (size_t)t7_param_int(&p, "max_history", (int64_t)tp.max_history);
  s.n_ignore = t7_param_int_array(&p, "ignore_ids", s.ignore, MAX_IGNORE);
  t7_params_fini(&p);

  t7_expand_home(out_raw, out_dir, sizeof(out_dir));
  if (t7_mkdirs(out_dir) != 0) {
    RCUTILS_LOG_ERROR_NAMED(LOG, "cannot create %s", out_dir);
  }
  snprintf(s.csv_path, sizeof(s.csv_path), "%s/%s", out_dir, csv);
  RCUTILS_LOG_INFO_NAMED(LOG, "writing confirmed targets to %s", s.csv_path);

  tb4_grid_init_empty(&s.grid);
  if (tb4_tracker_init(&s.tracker, tp) != 0) {
    RCUTILS_LOG_FATAL_NAMED(LOG, "out of memory");
    return 1;
  }
  visualization_msgs__msg__MarkerArray__init(&s.out);

  rclc_executor_t ex = rclc_executor_get_zero_initialized_executor();
  T7_CHECK(rclc_executor_init(&ex, &support.context, 4, &alloc));
  t7_tf_listener_init(&s.tf, &node, &ex, tf_topic, tfs_topic, 10.0, 0.1);

  rcl_subscription_t sub_map = rcl_get_zero_initialized_subscription();
  rmw_qos_profile_t latched = t7_qos(1, true, true);
  T7_CHECK(rclc_subscription_init(&sub_map, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, OccupancyGrid),
                                  map_topic, &latched));
  nav_msgs__msg__OccupancyGrid map_msg;
  nav_msgs__msg__OccupancyGrid__init(&map_msg);
  T7_CHECK(rclc_executor_add_subscription_with_context(&ex, &sub_map, &map_msg, map_cb, &s, ON_NEW_DATA));

  rcl_subscription_t sub_det = rcl_get_zero_initialized_subscription();
  T7_CHECK(rclc_subscription_init(&sub_det, &node,
                                  ROSIDL_GET_MSG_TYPE_SUPPORT(aruco_opencv_msgs, msg, ArucoDetection),
                                  det_topic, &rmw_qos_profile_sensor_data));
  aruco_opencv_msgs__msg__ArucoDetection det_msg;
  aruco_opencv_msgs__msg__ArucoDetection__init(&det_msg);
  T7_CHECK(rclc_executor_add_subscription_with_context(&ex, &sub_det, &det_msg, detection_cb, &s, ON_NEW_DATA));

  s.pub_targets = rcl_get_zero_initialized_publisher();
  T7_CHECK(rclc_publisher_init_default(&s.pub_targets, &node,
                                       ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, MarkerArray),
                                       "/team7/targets"));

  t7_spin(&ex, NULL);

  RCUTILS_LOG_INFO_NAMED(LOG, "shutting down (%" PRIu64 " raycast, %" PRIu64 " pnp estimates)",
                         s.stats[0], s.stats[1]);
  T7_IGNORE(rclc_executor_fini(&ex));
  T7_IGNORE(rcl_publisher_fini(&s.pub_targets, &node));
  T7_IGNORE(rcl_subscription_fini(&sub_det, &node));
  T7_IGNORE(rcl_subscription_fini(&sub_map, &node));
  t7_tf_listener_fini(&s.tf, &node);
  aruco_opencv_msgs__msg__ArucoDetection__fini(&det_msg);
  nav_msgs__msg__OccupancyGrid__fini(&map_msg);
  visualization_msgs__msg__MarkerArray__fini(&s.out);
  tb4_tracker_free(&s.tracker);
  tb4_grid_free(&s.grid);
  T7_IGNORE(rcl_node_fini(&node));
  T7_IGNORE(rclc_support_fini(&support));
  return 0;
}
