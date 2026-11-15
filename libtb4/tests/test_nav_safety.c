#include <math.h>
#include <stdio.h>

#include "tb4/nav_safety.h"

static int failures, checks;
#define CHECK(expr) do { ++checks; if (!(expr)) { ++failures; \
  fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); } } while (0)

static tb4_nav_safety_input input(double now)
{
  return (tb4_nav_safety_input){.active_valid = true, .active = true,
    .command_valid = true, .odom_valid = true, .scan_valid = true,
    .active_at = now, .command_at = now, .odom_at = now, .scan_at = now, .v = 0.1};
}

static tb4_nav_safety_report step(tb4_nav_safety *s, tb4_nav_safety_input in, double now)
{
  in.active_at = in.command_at = in.odom_at = in.scan_at = now;
  return tb4_nav_safety_step(s, &in, now);
}

static void test_progress(void)
{
  tb4_nav_safety s;
  CHECK(tb4_nav_safety_init(&s, tb4_nav_safety_defaults()));
  tb4_nav_safety_input in = input(0);
  for (int i = 0; i < 30; ++i) {
    in.x = (double)i * 0.06;
    CHECK(step(&s, in, i).status == TB4_NAV_CLEAR);
  }
  tb4_nav_safety_reset(&s);
  in = input(0); in.v = 0; in.w = 0.3;
  for (int i = 0; i < 30; ++i) {
    in.yaw = remainder(3.0 + (double)i * 0.2, 2*M_PI);
    CHECK(step(&s, in, i).status == TB4_NAV_CLEAR);
  }
  tb4_nav_safety_reset(&s);
  in = input(0);
  CHECK(step(&s, in, 0).status == TB4_NAV_CLEAR);
  in.yaw = 1.0; /* rotation alone cannot satisfy translation progress */
  tb4_nav_safety_report r = step(&s, in, 8);
  CHECK(r.status == TB4_NAV_STUCK && r.reason == TB4_NAV_REASON_NO_LINEAR_PROGRESS);
  CHECK(r.observed_seconds == 8.0);
  in.v = 0; in.w = 0;
  CHECK(step(&s, in, 9).status == TB4_NAV_CLEAR); /* current condition, no latch/ack */
  CHECK(step(&s, in, 10).status == TB4_NAV_CLEAR);
  tb4_nav_safety_reset(&s);
  for (int i = 0; i < 30; ++i) CHECK(step(&s, in, i).status == TB4_NAV_CLEAR);
  in.w = 0.2;
  CHECK(step(&s, in, 30).status == TB4_NAV_CLEAR);
  r = step(&s, in, 38);
  CHECK(r.status == TB4_NAV_STUCK && r.reason == TB4_NAV_REASON_NO_ANGULAR_PROGRESS);
  in.yaw = 0.2;
  CHECK(step(&s, in, 39).status == TB4_NAV_CLEAR);
}

static void test_blocked_and_recovery(void)
{
  tb4_nav_safety s;
  CHECK(tb4_nav_safety_init(&s, tb4_nav_safety_defaults()));
  tb4_nav_safety_input in = input(0); in.obstacle = true;
  tb4_nav_safety_report r = step(&s, in, 0);
  CHECK(r.status == TB4_NAV_BLOCKED && r.reason == TB4_NAV_REASON_OBSTACLE);
  CHECK(step(&s, in, 4).observed_seconds == 4.0);
  in.obstacle = false;
  CHECK(step(&s, in, 4.5).status == TB4_NAV_CLEAR);
  in.obstacle = true;
  CHECK(step(&s, in, 5).status == TB4_NAV_BLOCKED);
  r = step(&s, in, 10);
  CHECK(r.status == TB4_NAV_STUCK && r.reason == TB4_NAV_REASON_PERSISTENT_OBSTACLE);
  CHECK(r.observed_seconds == 5.0);
  CHECK(step(&s, in, 11).observed_seconds == 6.0);
  in.obstacle = false;
  CHECK(step(&s, in, 12).status == TB4_NAV_CLEAR); /* clear without reset or ack */
  CHECK(step(&s, in, 20).status == TB4_NAV_STUCK);
  in.x = 0.1;
  CHECK(step(&s, in, 21).status == TB4_NAV_CLEAR); /* physical progress clears stuck */
  for (int i = 0; i < 8; ++i) {
    double at = 30.0 + (double)i * 10.0;
    in.active = false;
    CHECK(step(&s, in, at).status == TB4_NAV_IDLE);
    in.active = true; in.obstacle = true;
    CHECK(step(&s, in, at + 1).status == TB4_NAV_BLOCKED);
    CHECK(step(&s, in, at + 6).status == TB4_NAV_STUCK); /* no retry limit / FAILED */
  }
  in.active = false;
  CHECK(step(&s, in, 110).reason == TB4_NAV_REASON_INACTIVE);
  in.active = true;
  CHECK(step(&s, in, 111).observed_seconds == 0.0);
}

static void test_intent_switches(void)
{
  tb4_nav_safety s;
  CHECK(tb4_nav_safety_init(&s, tb4_nav_safety_defaults()));
  tb4_nav_safety_input in = input(0);
  /* Intent switches alone must not hide a physically jammed robot. */
  for (int t = 0; t <= 40; ++t) {
    in.v = t % 2 ? 0.0 : 0.1;
    in.w = t % 2 ? 0.3 : 0.0;
    tb4_nav_safety_report r = step(&s, in, t);
    CHECK(r.status == (t < 8 ? TB4_NAV_CLEAR : TB4_NAV_STUCK));
    if (t >= 8) CHECK(r.observed_seconds == (double)t);
  }

  /* Real progress in each previous intent survives repeated drive/turn switches. */
  tb4_nav_safety_reset(&s);
  in = input(0);
  CHECK(step(&s, in, 0).status == TB4_NAV_CLEAR);
  for (int t = 1; t <= 30; ++t) {
    if (t % 2) { in.x += 0.06; in.v = 0.0; in.w = 0.3; }
    else { in.yaw = remainder(in.yaw + 0.2, 2 * M_PI); in.v = 0.1; in.w = 0.0; }
    CHECK(step(&s, in, t).status == TB4_NAV_CLEAR);
  }

  /* Sub-threshold steps at 10 Hz must accumulate across intent switches. */
  tb4_nav_safety_reset(&s);
  in = input(0);
  CHECK(step(&s, in, 0).status == TB4_NAV_CLEAR);
  for (int t = 1; t <= 200; ++t) {
    if (t % 2) { in.x += 0.01; in.v = 0.0; in.w = 0.3; }
    else { in.yaw = remainder(in.yaw + 0.03, 2 * M_PI); in.v = 0.1; in.w = 0.0; }
    CHECK(step(&s, in, (double)t * 0.1).status == TB4_NAV_CLEAR);
  }

  /* Old rotation while translation was requested cannot become progress merely
   * by switching to rotation, nor can old translation count after switching back. */
  tb4_nav_safety_reset(&s);
  in = input(0);
  CHECK(step(&s, in, 0).status == TB4_NAV_CLEAR);
  in.yaw = 1.0;
  CHECK(step(&s, in, 7).status == TB4_NAV_CLEAR);
  in.v = 0.0; in.w = 0.3;
  CHECK(step(&s, in, 7.5).status == TB4_NAV_CLEAR);
  CHECK(step(&s, in, 8).status == TB4_NAV_STUCK);
  in.x = 0.2;
  CHECK(step(&s, in, 9).status == TB4_NAV_STUCK);
  in.v = 0.1; in.w = 0.0;
  CHECK(step(&s, in, 10).status == TB4_NAV_STUCK);
  in.x += 0.06;
  CHECK(step(&s, in, 11).status == TB4_NAV_CLEAR);
}

static void test_obstacles_with_progress(void)
{
  tb4_nav_safety s;
  CHECK(tb4_nav_safety_init(&s, tb4_nav_safety_defaults()));
  tb4_nav_safety_input in = input(0); in.obstacle = true;
  for (int t = 0; t <= 20; ++t) {
    in.x = (double)t * 0.1;
    tb4_nav_safety_report r = step(&s, in, t);
    CHECK(r.status == TB4_NAV_BLOCKED);
    CHECK(r.observed_seconds == 0.0);
  }
  CHECK(step(&s, in, 24).status == TB4_NAV_BLOCKED);
  CHECK(step(&s, in, 25).status == TB4_NAV_STUCK); /* actual progress has stopped */
  in.x += 0.1;
  CHECK(step(&s, in, 26).status == TB4_NAV_BLOCKED);

  tb4_nav_safety_reset(&s);
  in = input(0); in.obstacle = true; in.v = 0.0; in.w = 0.3;
  for (int t = 0; t <= 20; ++t) {
    in.yaw = remainder(3.0 + (double)t * 0.2, 2 * M_PI);
    CHECK(step(&s, in, t).status == TB4_NAV_BLOCKED);
  }
  /* Angular motion cannot hide failure to translate under a translation intent. */
  tb4_nav_safety_reset(&s);
  in = input(0); in.obstacle = true;
  CHECK(step(&s, in, 0).status == TB4_NAV_BLOCKED);
  in.yaw = 1.0;
  CHECK(step(&s, in, 5).status == TB4_NAV_STUCK);
}

static void test_freshness(void)
{
  tb4_nav_safety s;
  CHECK(tb4_nav_safety_init(&s, tb4_nav_safety_defaults()));
  tb4_nav_safety_input in = input(0);
  CHECK(tb4_nav_safety_step(&s, &in, 0).status == TB4_NAV_CLEAR);
  for (int which = 0; which < 4; ++which) {
    in = input(10);
    if (which == 0) in.active_at = 0;
    if (which == 1) in.command_at = 0;
    if (which == 2) in.odom_at = 0;
    if (which == 3) in.scan_at = 0;
    tb4_nav_safety_report r = tb4_nav_safety_step(&s, &in, 10);
    const tb4_nav_reason reasons[] = {TB4_NAV_REASON_ACTIVE_INPUT, TB4_NAV_REASON_COMMAND_INPUT,
      TB4_NAV_REASON_ODOM_INPUT, TB4_NAV_REASON_SCAN_INPUT};
    CHECK(r.status == TB4_NAV_INPUT_STALE && r.reason == reasons[which]);
  }
  in = input(11);
  CHECK(tb4_nav_safety_step(&s, &in, 11).status == TB4_NAV_CLEAR);
  CHECK(step(&s, in, 18).status == TB4_NAV_CLEAR); /* outage wasn't counted as stuck */
  in.v = NAN;
  CHECK(step(&s, in, 19).status == TB4_NAV_INPUT_STALE);
  in = input(20); in.active = false; in.scan_valid = false;
  CHECK(tb4_nav_safety_step(&s, &in, 20).status == TB4_NAV_IDLE);
  in = input(21); in.scan_at = 22;
  CHECK(tb4_nav_safety_step(&s, &in, 21).status == TB4_NAV_INPUT_STALE);
  tb4_nav_safety_params p = tb4_nav_safety_defaults(); p.lookahead = NAN;
  CHECK(!tb4_nav_safety_init(&s, p));
}

static void test_geometry(void)
{
  tb4_nav_safety_params p = tb4_nav_safety_defaults();
  tb4_nav_point front = {0.30, 0}, back = {-0.30, 0}, side = {0, 0.35}, near = {0, 0.23};
  CHECK(tb4_nav_safety_obstacle(&p, 0.1, 0, &front, 1));
  CHECK(!tb4_nav_safety_obstacle(&p, 0.1, 0, &back, 1));
  CHECK(tb4_nav_safety_obstacle(&p, -0.1, 0, &back, 1));
  CHECK(!tb4_nav_safety_obstacle(&p, 0.1, 0, &side, 1));
  CHECK(tb4_nav_safety_obstacle(&p, 0, 0.3, &near, 1));
  CHECK(!tb4_nav_safety_obstacle(&p, 0, 0.3, &front, 1));
  CHECK(!tb4_nav_safety_obstacle(&p, 0, 0, &near, 1));
  CHECK(!tb4_nav_safety_obstacle(&p, 0.1, 0, NULL, 0));
}

int main(void)
{
  test_progress(); test_blocked_and_recovery(); test_intent_switches();
  test_obstacles_with_progress(); test_freshness(); test_geometry();
  printf("nav_safety: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
