#include "tb4/nav_safety.h"

#include <math.h>
#include <string.h>

tb4_nav_safety_params tb4_nav_safety_defaults(void)
{
  return (tb4_nav_safety_params){
    .input_timeout = 1.0, .active_timeout = 2.0,
    .progress_timeout = 8.0, .blocked_timeout = 5.0,
    .progress_distance = 0.05, .progress_angle = 0.15,
    .min_linear = 0.02, .min_angular = 0.05,
    .robot_radius = 0.19, .safety_margin = 0.05, .lookahead = 1.2
  };
}

bool tb4_nav_safety_init(tb4_nav_safety *s, tb4_nav_safety_params p)
{
  const double positive[] = {p.input_timeout, p.active_timeout, p.progress_timeout,
    p.blocked_timeout, p.progress_distance, p.progress_angle, p.min_linear,
    p.min_angular, p.robot_radius, p.lookahead};
  if (!s) return false;
  for (size_t i = 0; i < sizeof(positive) / sizeof(positive[0]); ++i)
    if (!isfinite(positive[i]) || positive[i] <= 0.0) return false;
  if (!isfinite(p.safety_margin) || p.safety_margin < 0.0) return false;
  memset(s, 0, sizeof(*s));
  s->params = p;
  return true;
}

void tb4_nav_safety_reset(tb4_nav_safety *s)
{
  tb4_nav_safety_params p = s->params;
  memset(s, 0, sizeof(*s));
  s->params = p;
}

static bool fresh(bool valid, double at, double now, double timeout)
{
  return valid && isfinite(at) && at <= now && now - at < timeout;
}

static tb4_nav_safety_report report(tb4_nav_status status, tb4_nav_reason reason, double elapsed)
{
  return (tb4_nav_safety_report){.status = status, .reason = reason, .observed_seconds = elapsed};
}

static tb4_nav_safety_report stale(tb4_nav_safety *s, tb4_nav_reason reason)
{
  s->tracking = s->blocking = false;
  return report(TB4_NAV_INPUT_STALE, reason, 0.0);
}

tb4_nav_safety_report tb4_nav_safety_step(tb4_nav_safety *s,
                                        const tb4_nav_safety_input *in, double now)
{
  const tb4_nav_safety_params *p = &s->params;
  if (!isfinite(now) || !fresh(in->active_valid, in->active_at, now, p->active_timeout)) {
    return stale(s, TB4_NAV_REASON_ACTIVE_INPUT);
  }
  if (!in->active) {
    s->tracking = s->blocking = false;
    return report(TB4_NAV_IDLE, TB4_NAV_REASON_INACTIVE, 0.0);
  }
  if (!fresh(in->command_valid, in->command_at, now, p->input_timeout))
    return stale(s, TB4_NAV_REASON_COMMAND_INPUT);
  if (!fresh(in->odom_valid, in->odom_at, now, p->input_timeout))
    return stale(s, TB4_NAV_REASON_ODOM_INPUT);
  if (!fresh(in->scan_valid, in->scan_at, now, p->input_timeout))
    return stale(s, TB4_NAV_REASON_SCAN_INPUT);
  if (!isfinite(in->v) || !isfinite(in->w)
      || !isfinite(in->x) || !isfinite(in->y) || !isfinite(in->yaw)) {
    return stale(s, TB4_NAV_REASON_INVALID_VALUE);
  }

  /* Translating must make positional progress, even if its wheels keep turning.
   * A pure turn instead requires angular progress, including across +/- pi. */
  int kind = fabs(in->v) >= p->min_linear ? 1 : fabs(in->w) >= p->min_angular ? 2 : 0;
  if (!kind) {
    s->tracking = s->blocking = false;
    return report(TB4_NAV_CLEAR, TB4_NAV_REASON_NONE, 0.0);
  }

  if (!s->tracking) {
    s->tracking = true;
    s->motion_kind = kind;
    s->progress_at = now;
    s->x = in->x; s->y = in->y; s->yaw = in->yaw;
    s->linear_dx = s->linear_dy = s->angular_delta = 0.0;
  }
  /* Attribute motion since the last sample to the previous intent. Changing
   * intent does not renew the deadline. Keep partial motion across switches
   * so frequent small drive/turn steps can still demonstrate real progress. */
  if (s->motion_kind == 1) {
    s->linear_dx += in->x - s->x;
    s->linear_dy += in->y - s->y;
  } else {
    s->angular_delta += remainder(in->yaw - s->yaw, 2.0 * M_PI);
  }
  bool progress = s->motion_kind == 1
    ? hypot(s->linear_dx, s->linear_dy) >= p->progress_distance
    : fabs(s->angular_delta) >= p->progress_angle;
  s->x = in->x; s->y = in->y; s->yaw = in->yaw;
  s->motion_kind = kind;
  if (progress) {
    s->progress_at = now;
    s->linear_dx = s->linear_dy = s->angular_delta = 0.0;
    if (s->blocking) s->blocked_at = now;
  }
  if (in->obstacle) {
    if (!s->blocking) { s->blocking = true; s->blocked_at = now; }
    double elapsed = now - s->blocked_at;
    if (elapsed >= p->blocked_timeout)
      return report(TB4_NAV_STUCK, TB4_NAV_REASON_PERSISTENT_OBSTACLE, elapsed);
    return report(TB4_NAV_BLOCKED, TB4_NAV_REASON_OBSTACLE, elapsed);
  }
  /* A cleared obstacle starts a fresh unobstructed observation interval. */
  if (s->blocking) {
    s->progress_at = now;
    s->x = in->x; s->y = in->y; s->yaw = in->yaw;
    s->linear_dx = s->linear_dy = s->angular_delta = 0.0;
  }
  s->blocking = false;
  if (now - s->progress_at >= p->progress_timeout) {
    return report(TB4_NAV_STUCK, kind == 1 ? TB4_NAV_REASON_NO_LINEAR_PROGRESS
                  : TB4_NAV_REASON_NO_ANGULAR_PROGRESS, now - s->progress_at);
  }
  return report(TB4_NAV_CLEAR, TB4_NAV_REASON_NONE, 0.0);
}

bool tb4_nav_safety_obstacle(const tb4_nav_safety_params *p, double v, double w,
                             const tb4_nav_point *points, size_t count)
{
  if (!isfinite(v) || !isfinite(w)) return true;
  if (fabs(v) < p->min_linear && fabs(w) < p->min_angular) return false;
  double radius = p->robot_radius + p->safety_margin;
  for (size_t i = 0; i < count; ++i) {
    double x = points[i].x, y = points[i].y;
    if (!isfinite(x) || !isfinite(y)) return true;
    if (hypot(x, y) <= radius) return true;
    double along = v >= 0.0 ? x : -x;
    if (fabs(v) >= p->min_linear && along >= 0.0
        && along <= radius + fabs(v) * p->lookahead && fabs(y) <= radius) return true;
  }
  return false;
}

const char *tb4_nav_status_name(tb4_nav_status status)
{
  switch (status) {
    case TB4_NAV_IDLE: return "IDLE";
    case TB4_NAV_CLEAR: return "CLEAR";
    case TB4_NAV_BLOCKED: return "BLOCKED";
    case TB4_NAV_STUCK: return "STUCK";
    case TB4_NAV_INPUT_STALE: return "INPUT_STALE";
    default: return "INPUT_STALE";
  }
}

const char *tb4_nav_reason_name(tb4_nav_reason reason)
{
  switch (reason) {
    case TB4_NAV_REASON_NONE: return "NONE";
    case TB4_NAV_REASON_INACTIVE: return "INACTIVE";
    case TB4_NAV_REASON_ACTIVE_INPUT: return "ACTIVE_INPUT";
    case TB4_NAV_REASON_COMMAND_INPUT: return "COMMAND_INPUT";
    case TB4_NAV_REASON_ODOM_INPUT: return "ODOM_INPUT";
    case TB4_NAV_REASON_SCAN_INPUT: return "SCAN_INPUT";
    case TB4_NAV_REASON_INVALID_VALUE: return "INVALID_VALUE";
    case TB4_NAV_REASON_OBSTACLE: return "OBSTACLE";
    case TB4_NAV_REASON_PERSISTENT_OBSTACLE: return "PERSISTENT_OBSTACLE";
    case TB4_NAV_REASON_NO_LINEAR_PROGRESS: return "NO_LINEAR_PROGRESS";
    case TB4_NAV_REASON_NO_ANGULAR_PROGRESS: return "NO_ANGULAR_PROGRESS";
    default: return "INVALID_VALUE";
  }
}
