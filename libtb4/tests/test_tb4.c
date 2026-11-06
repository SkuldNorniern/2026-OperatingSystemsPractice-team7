#include <math.h>
#include <stdio.h>
#include <string.h>

#include "tb4/explore.h"
#include "tb4/geom.h"
#include "tb4/grid.h"
#include "tb4/markers.h"
#include "tb4/tf.h"

static int g_failures, g_checks;
#define CHECK(cond)                                                          \
  do {                                                                       \
    g_checks++;                                                              \
    if (!(cond)) {                                                           \
      g_failures++;                                                          \
      fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
    }                                                                        \
  } while (0)
#define NEAR(a, b, eps) CHECK(fabs((a) - (b)) < (eps))

static void room(tb4_grid *g)
{
  tb4_grid_meta m = {200, 200, 0.05, -5.0, -5.0};
  tb4_grid_init(g, m, 0);
  for (uint32_t cy = 0; cy < 200; cy++) {
    uint32_t cx;
    tb4_grid_world_to_cell(g, 2.01, 0.0, &cx, &(uint32_t){0});
    tb4_grid_set(g, cx, cy, 100);
    tb4_grid_set(g, cx + 1, cy, 100);
  }
}

static void test_world_cell_roundtrip(void)
{
  tb4_grid g;
  room(&g);
  uint32_t cx, cy;
  CHECK(tb4_grid_world_to_cell(&g, 0.12, -0.33, &cx, &cy));
  double x, y;
  tb4_grid_cell_to_world(&g, cx, cy, &x, &y);
  NEAR(x, 0.125, 1e-9);
  NEAR(y, -0.325, 1e-9);
  CHECK(!tb4_grid_world_to_cell(&g, 5.01, 0.0, &cx, &cy));
  CHECK(!tb4_grid_world_to_cell(&g, -5.01, 0.0, &cx, &cy));
  tb4_grid_free(&g);
}

static void test_raycast(void)
{
  tb4_grid g;
  room(&g);
  tb4_ray_hit h;
  CHECK(tb4_grid_raycast(&g, 0, 0, 1, 0, 8, 65, &h));
  NEAR(h.x, 2.0, 1e-9);
  NEAR(h.range, 2.0, 1e-9);
  CHECK(tb4_grid_raycast(&g, 0, 0, 1, 1, 8, 65, &h));
  NEAR(h.x, 2.0, 1e-9);
  NEAR(h.y, 2.0, 1e-9);
  CHECK(!tb4_grid_raycast(&g, 0, 0, -1, 0, 8, 65, &h));
  CHECK(!tb4_grid_raycast(&g, 0, 0, 1, 0, 1.5, 65, &h));
  CHECK(!tb4_grid_raycast(&g, 0, 0, 0, 0, 8, 65, &h));
  for (uint32_t cx = 100; cx < 110; cx++) {
    tb4_grid_set(&g, cx, 100, TB4_UNKNOWN);
  }
  CHECK(tb4_grid_raycast(&g, 0, 0, 1, 0, 8, 65, &h));
  tb4_grid_free(&g);
}

static tb4_iso iso(double x, double y, double z, double yaw)
{
  return tb4_iso_make(x, y, z, tb4_quat_from_yaw(yaw));
}
static int64_t sec(double s) { return (int64_t)(s * 1e9); }

static tb4_tf_buffer *tree(void)
{
  tb4_tf_buffer *b = tb4_tf_create(10.0, 0.05);
  tb4_tf_insert(b, "map", "odom", sec(1), iso(1, 0, 0, 0), false);
  tb4_tf_insert(b, "map", "odom", sec(2), iso(1, 0, 0, 0), false);
  tb4_tf_insert(b, "odom", "base_link", sec(1), iso(0, 0, 0, 0), false);
  tb4_tf_insert(b, "odom", "base_link", sec(2), iso(2, 0, 0, M_PI / 2), false);
  tb4_tf_insert(b, "base_link", "camera", 0, iso(0.1, 0, 0.2, 0), true);
  tb4_tf_insert(b, "/base_link", "lidar", 0, iso(-0.1, 0, 0.1, 0), true);
  return b;
}

static void test_tf_chain_and_interpolation(void)
{
  tb4_tf_buffer *b = tree();
  tb4_iso t;
  CHECK(tb4_tf_lookup(b, "map", "camera", sec(1.5), &t) == TB4_TF_OK);
  tb4_vec3 p = tb4_iso_apply(t, (tb4_vec3){0, 0, 0});
  double c = cos(M_PI / 4) * 0.1;
  NEAR(p.x, 2.0 + c, 1e-9);
  NEAR(p.y, c, 1e-9);
  NEAR(p.z, 0.2, 1e-9);
  tb4_tf_destroy(b);
}

static void test_tf_inverse_and_sibling(void)
{
  tb4_tf_buffer *b = tree();
  tb4_iso a, c, s, m;
  CHECK(tb4_tf_lookup(b, "camera", "map", sec(2), &a) == TB4_TF_OK);
  CHECK(tb4_tf_lookup(b, "map", "camera", sec(2), &c) == TB4_TF_OK);
  NEAR(tb4_vec3_norm(tb4_iso_mul(a, c).t), 0.0, 1e-9);
  CHECK(tb4_tf_lookup(b, "lidar", "camera", sec(2), &s) == TB4_TF_OK);
  NEAR(s.t.x, 0.2, 1e-9);
  NEAR(s.t.z, 0.1, 1e-9);
  CHECK(tb4_tf_lookup(b, "odom", "camera", sec(2), &m) == TB4_TF_OK);
  NEAR(m.t.x, 2.0, 1e-9);
  NEAR(m.t.y, 0.1, 1e-9);
  tb4_tf_destroy(b);
}

static void test_tf_errors(void)
{
  tb4_tf_buffer *b = tree();
  tb4_iso t;
  CHECK(tb4_tf_lookup(b, "map", "camera", sec(3), &t) == TB4_TF_EXTRAPOLATION);
  CHECK(tb4_tf_lookup(b, "map", "camera", sec(2.04), &t) == TB4_TF_OK);
  CHECK(tb4_tf_lookup(b, "map", "camera", TB4_TF_LATEST, &t) == TB4_TF_OK);
  CHECK(tb4_tf_lookup(b, "map", "nope", sec(1), &t) == TB4_TF_UNKNOWN_FRAME);
  tb4_tf_insert(b, "world", "other", 0, tb4_iso_identity(), true);
  CHECK(tb4_tf_lookup(b, "map", "other", sec(1), &t) == TB4_TF_NOT_CONNECTED);
  tb4_tf_destroy(b);

  b = tb4_tf_create(1.0, 0.0);
  for (int i = 0; i < 100; i++) {
    tb4_tf_insert(b, "odom", "base_link", sec(i * 0.1), tb4_iso_identity(), false);
  }
  CHECK(tb4_tf_lookup(b, "odom", "base_link", sec(5.0), &t) == TB4_TF_EXTRAPOLATION);
  CHECK(tb4_tf_lookup(b, "odom", "base_link", sec(9.5), &t) == TB4_TF_OK);
  tb4_tf_destroy(b);
}

static void partial_map(tb4_grid *g)
{
  tb4_grid_meta m = {40, 40, 0.1, 0.0, 0.0};
  tb4_grid_init(g, m, TB4_UNKNOWN);
  for (uint32_t y = 0; y < 20; y++) {
    for (uint32_t x = 0; x < 20; x++) {
      tb4_grid_set(g, x, y, 0);
    }
  }
  for (uint32_t i = 0; i < 20; i++) {
    tb4_grid_set(g, 0, i, 100);
    tb4_grid_set(g, i, 0, 100);
  }
}

static void test_frontiers(void)
{
  tb4_grid g;
  partial_map(&g);
  tb4_explore_params p = tb4_explore_default_params();
  tb4_frontier_set s;
  tb4_frontier_set_init(&s);
  CHECK(tb4_find_frontiers(&g, 0.5, 0.5, &p, &s) == 1);
  CHECK(s.items[0].cells == 37);
  uint32_t cx, cy;
  CHECK(tb4_grid_world_to_cell(&g, s.items[0].gx, s.items[0].gy, &cx, &cy));

  CHECK(tb4_select_goal(&s, NULL, 0, &p) != NULL);
  double bl[1][2] = {{s.items[0].gx, s.items[0].gy}};
  CHECK(tb4_select_goal(&s, (const double(*)[2])bl, 1, &p) == NULL);
  double gx = s.items[0].gx, gy = s.items[0].gy;
  tb4_find_frontiers(&g, gx, gy, &p, &s);
  CHECK(tb4_select_goal(&s, NULL, 0, &p) == NULL);

  for (uint32_t d = 17; d < 20; d++) {
    for (uint32_t e = 17; e < 20; e++) {
      tb4_grid_set(&g, d, e, 100);
    }
  }
  p.min_frontier_cells = 3;
  CHECK(tb4_find_frontiers(&g, 1.8, 0.5, &p, &s) == 2);
  CHECK(s.items[0].gx > 1.8);
  CHECK(s.items[0].score >= s.items[1].score);

  tb4_grid k;
  tb4_grid_init(&k, (tb4_grid_meta){10, 10, 0.1, 0, 0}, 0);
  CHECK(tb4_find_frontiers(&k, 0, 0, &p, &s) == 0);
  tb4_grid_free(&k);
  tb4_frontier_set_free(&s);
  tb4_grid_free(&g);
}

static void wall_map(tb4_grid *g)
{
  tb4_grid_init(g, (tb4_grid_meta){140, 140, 0.05, -3.0, -3.0}, 0);
  uint32_t cx, cy;
  tb4_grid_world_to_cell(g, 3.01, 0.0, &cx, &cy);
  for (uint32_t y = 0; y < 140; y++) {
    tb4_grid_set(g, cx, y, 100);
  }
}

static tb4_observation obs(double pnp_range)
{
  tb4_observation o = {7, {0, 0, 0.25}, {1, 0, 0}, pnp_range};
  return o;
}

static void test_locate(void)
{
  tb4_grid g;
  wall_map(&g);
  tb4_locate_params p = tb4_locate_default_params();
  tb4_estimate e;
  tb4_observation o = obs(2.0);
  CHECK(tb4_locate(&g, &o, &p, &e));
  CHECK(e.source == TB4_SRC_RAYCAST);
  NEAR(e.x, 3.0, 1e-9);
  NEAR(e.y, 0.0, 1e-9);

  tb4_locate_params strict = p;
  strict.range_ratio_gate = 1.2;
  strict.allow_pnp_fallback = false;
  o = obs(1.0);
  CHECK(!tb4_locate(&g, &o, &strict, &e));

  o = obs(2.0);
  o.bearing[0] = 0.6;
  o.bearing[2] = 0.8 - 1e-9;
  CHECK(tb4_locate(NULL, &o, &p, &e));
  CHECK(e.source == TB4_SRC_PNP);
  NEAR(e.x, 1.2, 1e-6);
  o.bearing[0] = 0;
  o.bearing[2] = 1;
  CHECK(!tb4_locate(NULL, &o, &p, &e));
  tb4_grid_free(&g);
}

static void test_tracker(void)
{
  tb4_tracker t;
  tb4_track_params tp = tb4_track_default_params();
  tp.min_views = 3;
  tb4_tracker_init(&t, tp);
  tb4_estimate e = {0, 0, TB4_SRC_RAYCAST};
#define ADD(id, X, Y) (e.x = (X), e.y = (Y), tb4_tracker_add(&t, (id), &e))
  ADD(1, 1.0, 1.0);
  ADD(1, 1.02, 0.98);
  tb4_target tg;
  CHECK(tb4_tracker_target(&t, 1, &tg) && !tg.confirmed);
  ADD(1, 5.0, 5.0);
  ADD(1, 0.99, 1.01);
  CHECK(tb4_tracker_target(&t, 1, &tg) && tg.confirmed);
  NEAR(tg.x, 1.01, 0.02);
  NEAR(tg.y, 1.005, 0.02);
  for (int i = 0; i < 10; i++) {
    ADD(2, (double)i, 0.0);
  }
  CHECK(tb4_tracker_target(&t, 2, &tg) && !tg.confirmed);
  tb4_target all[8], conf[8];
  size_t n = tb4_tracker_targets(&t, all, 8), nc = 0;
  CHECK(n == 2);
  for (size_t i = 0; i < n; i++) {
    if (all[i].confirmed) {
      conf[nc++] = all[i];
    }
  }
  CHECK(nc == 1);
  char csv[128];
  tb4_targets_csv(conf, nc, csv, sizeof(csv));
  CHECK(strcmp(csv, "id,x,y\n1,1.010,1.005\n") == 0);
  tb4_tracker_free(&t);

  tp.max_history = 4;
  tb4_tracker_init(&t, tp);
  for (int i = 0; i < 10; i++) {
    ADD(3, (double)i, 0.0);
  }
  CHECK(tb4_tracker_target(&t, 3, &tg) && tg.views == 4);
  NEAR(tg.x, 7.5, 1e-9);
  tb4_tracker_free(&t);
#undef ADD
}

int main(void)
{
  struct { const char *name; void (*fn)(void); } tests[] = {
    {"world_cell_roundtrip", test_world_cell_roundtrip},
    {"raycast", test_raycast},
    {"tf_chain_and_interpolation", test_tf_chain_and_interpolation},
    {"tf_inverse_and_sibling", test_tf_inverse_and_sibling},
    {"tf_errors", test_tf_errors},
    {"frontiers", test_frontiers},
    {"locate", test_locate},
    {"tracker", test_tracker},
  };
  for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
    int before = g_failures;
    tests[i].fn();
    printf("%-30s %s\n", tests[i].name, g_failures == before ? "ok" : "FAILED");
  }
  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
