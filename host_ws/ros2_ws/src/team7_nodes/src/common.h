#ifndef TEAM7_COMMON_H
#define TEAM7_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcutils/logging_macros.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/transform.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <tf2_msgs/msg/tf_message.h>
#include <visualization_msgs/msg/marker.h>

#include "tb4/geom.h"
#include "tb4/grid.h"
#include "tb4/tf.h"

/* (void) cast doesnt silence gcc warn_unused_result */
#define T7_IGNORE(expr)          \
  do {                           \
    rcl_ret_t t7_ig_ = (expr);   \
    (void)t7_ig_;                \
  } while (0)

#define T7_CHECK(expr)                                                                  \
  do {                                                                                  \
    rcl_ret_t t7_rc_ = (expr);                                                          \
    if (t7_rc_ != RCL_RET_OK) {                                                         \
      RCUTILS_LOG_FATAL("%s failed (%d): %s", #expr, (int)t7_rc_, rcl_get_error_string().str); \
      rcl_reset_error();                                                                \
      exit(1);                                                                          \
    }                                                                                   \
  } while (0)

typedef struct {
  rcl_params_t *ov;
  char names[3][128];
} t7_params;

void t7_params_init(t7_params *p, rclc_support_t *support, const rcl_node_t *node);
void t7_params_fini(t7_params *p);
double t7_param_double(t7_params *p, const char *name, double def);
int64_t t7_param_int(t7_params *p, const char *name, int64_t def);
bool t7_param_bool(t7_params *p, const char *name, bool def);
const char *t7_param_string(t7_params *p, const char *name, const char *def, char *out, size_t cap);
size_t t7_param_int_array(t7_params *p, const char *name, int64_t *out, size_t max);

rmw_qos_profile_t t7_qos(size_t depth, bool reliable, bool transient_local);

typedef struct {
  tb4_tf_buffer *buf;
  rcl_subscription_t sub_dyn, sub_static;
  tf2_msgs__msg__TFMessage msg_dyn, msg_static;
} t7_tf_listener;

void t7_tf_listener_init(t7_tf_listener *l, rcl_node_t *node, rclc_executor_t *ex,
                         const char *tf_topic, const char *tf_static_topic,
                         double cache_s, double tolerance_s);
void t7_tf_listener_fini(t7_tf_listener *l, rcl_node_t *node);

tb4_iso t7_iso_from_transform(const geometry_msgs__msg__Transform *t);
tb4_iso t7_iso_from_pose(const geometry_msgs__msg__Pose *p);
static inline int64_t t7_stamp(const builtin_interfaces__msg__Time *t)
{
  return tb4_stamp(t->sec, t->nanosec);
}
bool t7_grid_from_msg(tb4_grid *g, const nav_msgs__msg__OccupancyGrid *m);

void t7_install_signal_handlers(void);
bool t7_interrupted(void);
void t7_spin(rclc_executor_t *ex, const volatile bool *stop);

void t7_expand_home(const char *in, char *out, size_t cap);
int t7_mkdirs(const char *path);
int t7_write_atomic(const char *path, const char *data, size_t len);

enum { T7_MARKER_SPHERE = 2, T7_MARKER_POINTS = 8, T7_MARKER_TEXT = 9 };
enum { T7_MARKER_ADD = 0, T7_MARKER_DELETEALL = 3 };

void t7_marker_setup(visualization_msgs__msg__Marker *m, const char *frame, const char *ns,
                     int32_t id, int32_t type, int32_t action);
void t7_marker_color(visualization_msgs__msg__Marker *m, float r, float g, float b, float a);

#endif
