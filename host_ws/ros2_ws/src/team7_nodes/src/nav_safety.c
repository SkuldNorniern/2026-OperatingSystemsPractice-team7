#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <geometry_msgs/msg/twist_stamped.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>

#include "common.h"
#include "tb4/nav_safety.h"

#define MAX_SCAN_POINTS 8192

static struct {
  tb4_nav_safety monitor;
  tb4_nav_safety_input input;
  tb4_nav_point points[MAX_SCAN_POINTS];
  size_t count;
  char base_frame[128], odom_frame[128];
  bool check_header_stamp;
  t7_tf_listener tf;
  rcl_publisher_t publisher;
  std_msgs__msg__String output;
  unsigned long long sequence;
  int last_status;
} S;

static double seconds(clockid_t clock)
{
  struct timespec ts;
  clock_gettime(clock, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static bool stamp_valid(const builtin_interfaces__msg__Time *stamp)
{
  if (!S.check_header_stamp) return true;
  double age = seconds(CLOCK_REALTIME) - (double)t7_stamp(stamp) * 1e-9;
  return t7_stamp(stamp) > 0 && age >= -0.2 && age < S.monitor.params.input_timeout;
}

static bool same_frame(const char *a, const char *b)
{
  if (!a || !b) return false;
  while (*a == '/') ++a;
  while (*b == '/') ++b;
  return *a && strcmp(a, b) == 0;
}

static void active_cb(const void *message, void *context)
{
  (void)context;
  S.input.active = ((const std_msgs__msg__Bool *)message)->data;
  S.input.active_at = seconds(CLOCK_MONOTONIC);
  S.input.active_valid = true;
}

static void command_cb(const void *message, void *context)
{
  (void)context;
  const geometry_msgs__msg__TwistStamped *m = message;
  S.input.command_valid = stamp_valid(&m->header.stamp)
    && same_frame(m->header.frame_id.data, S.base_frame);
  S.input.command_at = seconds(CLOCK_MONOTONIC);
  S.input.v = m->twist.linear.x;
  S.input.w = m->twist.angular.z;
  /* Only planar differential-drive commands are supported. */
  S.input.command_valid = S.input.command_valid
    && m->twist.linear.y == 0.0 && m->twist.linear.z == 0.0
    && m->twist.angular.x == 0.0 && m->twist.angular.y == 0.0;
}

static void odom_cb(const void *message, void *context)
{
  (void)context;
  const nav_msgs__msg__Odometry *m = message;
  const geometry_msgs__msg__Quaternion *q = &m->pose.pose.orientation;
  double norm = q->x*q->x + q->y*q->y + q->z*q->z + q->w*q->w;
  S.input.odom_valid = stamp_valid(&m->header.stamp)
    && same_frame(m->header.frame_id.data, S.odom_frame)
    && same_frame(m->child_frame_id.data, S.base_frame)
    && isfinite(norm) && fabs(norm - 1.0) < 0.01;
  S.input.odom_at = seconds(CLOCK_MONOTONIC);
  S.input.x = m->pose.pose.position.x;
  S.input.y = m->pose.pose.position.y;
  S.input.yaw = tb4_quat_yaw((tb4_quat){q->x, q->y, q->z, q->w});
}

static void scan_cb(const void *message, void *context)
{
  (void)context;
  const sensor_msgs__msg__LaserScan *m = message;
  S.input.scan_valid = false;
  S.count = 0;
  if (!stamp_valid(&m->header.stamp) || !m->ranges.size || m->ranges.size > MAX_SCAN_POINTS
      || !isfinite(m->angle_min) || !isfinite(m->angle_increment) || m->angle_increment == 0.0
      || !isfinite(m->range_min) || !isfinite(m->range_max)
      || m->range_min < 0.0 || m->range_max <= m->range_min) return;
  tb4_iso transform;
  if (same_frame(m->header.frame_id.data, S.base_frame)) transform = tb4_iso_identity();
  else if (!m->header.frame_id.data
      || tb4_tf_lookup(S.tf.buf, S.base_frame, m->header.frame_id.data,
                       t7_stamp(&m->header.stamp), &transform) != TB4_TF_OK) return;
  size_t usable = 0;
  for (size_t i = 0; i < m->ranges.size; ++i) {
    double range = m->ranges.data[i];
    /* +inf is the sensor's no-return indication; NaN/-inf/out-of-range are invalid. */
    if (isinf(range) && range > 0.0) { ++usable; continue; }
    if (!isfinite(range) || range < m->range_min || range > m->range_max) continue;
    ++usable;
    double angle = m->angle_min + (double)i * m->angle_increment;
    tb4_vec3 point = tb4_iso_apply(transform, (tb4_vec3){range*cos(angle), range*sin(angle), 0.0});
    if (!isfinite(point.x) || !isfinite(point.y)) return;
    S.points[S.count++] = (tb4_nav_point){point.x, point.y};
  }
  /* Mostly missing scans are not evidence of free space. */
  S.input.scan_valid = usable >= m->ranges.size - m->ranges.size / 5;
  S.input.scan_at = seconds(CLOCK_MONOTONIC);
}

static void tick(rcl_timer_t *timer, int64_t last_call)
{
  (void)timer; (void)last_call;
  double now = seconds(CLOCK_MONOTONIC);
  S.input.obstacle = tb4_nav_safety_obstacle(&S.monitor.params, S.input.v, S.input.w, S.points, S.count);
  tb4_nav_safety_report r = tb4_nav_safety_step(&S.monitor, &S.input, now);
  char json[512];
  snprintf(json, sizeof(json),
    "{\"version\":2,\"sequence\":%llu,\"monotonic_time\":%.6f,\"state\":\"%s\","
    "\"reason\":\"%s\",\"observed_seconds\":%.3f}",
    ++S.sequence, now, tb4_nav_status_name(r.status), tb4_nav_reason_name(r.reason), r.observed_seconds);
  if (rosidl_runtime_c__String__assign(&S.output.data, json))
    T7_IGNORE(rcl_publish(&S.publisher, &S.output, NULL));
  if ((int)r.status != S.last_status) {
    RCUTILS_LOG_INFO_NAMED("nav_safety", "%s (%s)", tb4_nav_status_name(r.status), tb4_nav_reason_name(r.reason));
    S.last_status = (int)r.status;
  }
}

int main(int argc, char **argv)
{
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rclc_support_t support;
  T7_CHECK(rclc_support_init(&support, argc, (const char *const *)argv, &alloc));
  rcl_node_t node = rcl_get_zero_initialized_node();
  T7_CHECK(rclc_node_init_default(&node, "nav_safety", "", &support));
  t7_install_signal_handlers();
  t7_params params;
  t7_params_init(&params, &support, &node);
  tb4_nav_safety_params p = tb4_nav_safety_defaults();
#define PARAM(name) p.name = t7_param_double(&params, #name, p.name)
  PARAM(input_timeout); PARAM(active_timeout); PARAM(progress_timeout); PARAM(blocked_timeout);
  PARAM(progress_distance); PARAM(progress_angle); PARAM(min_linear); PARAM(min_angular);
  PARAM(robot_radius); PARAM(safety_margin); PARAM(lookahead);
#undef PARAM
  if (!tb4_nav_safety_init(&S.monitor, p)) {
    RCUTILS_LOG_FATAL_NAMED("nav_safety", "invalid safety parameters");
    return 1;
  }
  S.check_header_stamp = t7_param_bool(&params, "check_header_stamp", true);
  t7_param_string(&params, "base_frame", "base_link", S.base_frame, sizeof(S.base_frame));
  t7_param_string(&params, "odom_frame", "odom", S.odom_frame, sizeof(S.odom_frame));
  t7_params_fini(&params);
  S.last_status = -1;

  rclc_executor_t ex = rclc_executor_get_zero_initialized_executor();
  T7_CHECK(rclc_executor_init(&ex, &support.context, 7, &alloc));
  t7_tf_listener_init(&S.tf, &node, &ex, "/tf", "/tf_static", 5.0, 0.2);
  rcl_subscription_t subs[4];
  std_msgs__msg__Bool active;
  geometry_msgs__msg__TwistStamped command;
  nav_msgs__msg__Odometry odom;
  sensor_msgs__msg__LaserScan scan;
  std_msgs__msg__Bool__init(&active);
  geometry_msgs__msg__TwistStamped__init(&command);
  nav_msgs__msg__Odometry__init(&odom); sensor_msgs__msg__LaserScan__init(&scan);
  rmw_qos_profile_t reliable = t7_qos(1, true, false), sensor = t7_qos(1, false, false);
#define SUB(index, pkg, type, topic, qos, storage, callback) do { \
    subs[index] = rcl_get_zero_initialized_subscription(); \
    T7_CHECK(rclc_subscription_init(&subs[index], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, msg, type), topic, &qos)); \
    T7_CHECK(rclc_executor_add_subscription_with_context(&ex, &subs[index], &storage, callback, NULL, ON_NEW_DATA)); \
  } while (0)
  SUB(0, std_msgs, Bool, "/team7/nav/active", reliable, active, active_cb);
  SUB(1, geometry_msgs, TwistStamped, "/team7/nav/intent", reliable, command, command_cb);
  SUB(2, nav_msgs, Odometry, "/odom", sensor, odom, odom_cb);
  SUB(3, sensor_msgs, LaserScan, "/scan", sensor, scan, scan_cb);
#undef SUB
  S.publisher = rcl_get_zero_initialized_publisher();
  T7_CHECK(rclc_publisher_init(&S.publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                               "/team7/nav/safety", &reliable));
  std_msgs__msg__String__init(&S.output);
  if (!rosidl_runtime_c__String__assign(&S.output.data, "")) return 1;
  rcl_timer_t timer = rcl_get_zero_initialized_timer();
  T7_CHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(100), tick, true));
  T7_CHECK(rclc_executor_add_timer(&ex, &timer));
  t7_spin(&ex, NULL);

  T7_IGNORE(rclc_executor_fini(&ex));
  T7_IGNORE(rcl_timer_fini(&timer));
  for (size_t i = 0; i < 4; ++i) T7_IGNORE(rcl_subscription_fini(&subs[i], &node));
  t7_tf_listener_fini(&S.tf, &node);
  T7_IGNORE(rcl_publisher_fini(&S.publisher, &node));
  std_msgs__msg__String__fini(&S.output);
  std_msgs__msg__Bool__fini(&active);
  geometry_msgs__msg__TwistStamped__fini(&command);
  nav_msgs__msg__Odometry__fini(&odom); sensor_msgs__msg__LaserScan__fini(&scan);
  T7_IGNORE(rcl_node_fini(&node)); T7_IGNORE(rclc_support_fini(&support));
  return 0;
}
