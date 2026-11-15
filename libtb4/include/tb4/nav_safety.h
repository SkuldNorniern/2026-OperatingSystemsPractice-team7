#ifndef TB4_NAV_SAFETY_H
#define TB4_NAV_SAFETY_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
  TB4_NAV_IDLE, TB4_NAV_CLEAR, TB4_NAV_BLOCKED, TB4_NAV_STUCK,
  TB4_NAV_INPUT_STALE
} tb4_nav_status;

typedef enum {
  TB4_NAV_REASON_NONE, TB4_NAV_REASON_INACTIVE, TB4_NAV_REASON_ACTIVE_INPUT,
  TB4_NAV_REASON_COMMAND_INPUT, TB4_NAV_REASON_ODOM_INPUT, TB4_NAV_REASON_SCAN_INPUT,
  TB4_NAV_REASON_INVALID_VALUE, TB4_NAV_REASON_OBSTACLE,
  TB4_NAV_REASON_PERSISTENT_OBSTACLE, TB4_NAV_REASON_NO_LINEAR_PROGRESS,
  TB4_NAV_REASON_NO_ANGULAR_PROGRESS
} tb4_nav_reason;

typedef struct {
  double input_timeout, active_timeout, progress_timeout, blocked_timeout;
  double progress_distance, progress_angle, min_linear, min_angular;
  double robot_radius, safety_margin, lookahead;
} tb4_nav_safety_params;

/* All *_at and now values use the same monotonic seconds clock.
 * Command is desired motion BEFORE downstream collision limiting.
 * active=false suspends detection during an intentional wait/recovery. */
typedef struct {
  bool active_valid, active, command_valid, odom_valid, scan_valid;
  double active_at, command_at, odom_at, scan_at;
  double v, w, x, y, yaw;
  bool obstacle;
} tb4_nav_safety_input;

typedef struct {
  tb4_nav_status status;
  tb4_nav_reason reason;
  double observed_seconds;
} tb4_nav_safety_report;

typedef struct {
  tb4_nav_safety_params params;
  bool tracking, blocking;
  int motion_kind;
  double progress_at, blocked_at, x, y, yaw;
  double linear_dx, linear_dy, angular_delta;
} tb4_nav_safety;

typedef struct { double x, y; } tb4_nav_point;

tb4_nav_safety_params tb4_nav_safety_defaults(void);
bool tb4_nav_safety_init(tb4_nav_safety *s, tb4_nav_safety_params p);
/* Local detector history only; there is no retry budget or failure latch. */
void tb4_nav_safety_reset(tb4_nav_safety *s);
tb4_nav_safety_report tb4_nav_safety_step(tb4_nav_safety *s,
                                        const tb4_nav_safety_input *in, double now);
/* Points must be in base_link. Circular footprint + swept straight corridor;
 * advisory only, not a replacement for Nav2 trajectory collision checking. */
bool tb4_nav_safety_obstacle(const tb4_nav_safety_params *p, double v, double w,
                             const tb4_nav_point *points, size_t count);
const char *tb4_nav_status_name(tb4_nav_status status);
const char *tb4_nav_reason_name(tb4_nav_reason reason);

#endif
