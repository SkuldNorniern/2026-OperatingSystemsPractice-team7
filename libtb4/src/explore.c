#include "tb4/explore.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

tb4_explore_params tb4_explore_default_params(void)
{
  tb4_explore_params p = {
    .free_threshold = 50,
    .min_frontier_cells = 8,
    .size_weight = 1.0,
    .distance_weight = 1.0,
    .blacklist_radius = 0.5,
    .min_goal_distance = 0.3,
  };
  return p;
}

void tb4_frontier_set_init(tb4_frontier_set *s)
{
  memset(s, 0, sizeof(*s));
}

void tb4_frontier_set_free(tb4_frontier_set *s)
{
  free(s->items);
  free(s->flag);
  free(s->seen);
  free(s->queue);
  free(s->members);
  tb4_frontier_set_init(s);
}

static int reserve_scratch(tb4_frontier_set *s, size_t n)
{
  if (n <= s->scratch_cells) {
    return 0;
  }
  uint8_t *f = realloc(s->flag, n);
  if (f) s->flag = f;
  uint8_t *v = realloc(s->seen, n);
  if (v) s->seen = v;
  uint32_t *q = realloc(s->queue, n * sizeof(uint32_t));
  if (q) s->queue = q;
  uint32_t *m = realloc(s->members, n * sizeof(uint32_t));
  if (m) s->members = m;
  if (!f || !v || !q || !m) {
    return -1;
  }
  s->scratch_cells = n;
  return 0;
}

static int push_frontier(tb4_frontier_set *s, const tb4_frontier *f)
{
  if (s->len == s->cap) {
    size_t cap = s->cap ? s->cap * 2 : 16;
    tb4_frontier *p = realloc(s->items, cap * sizeof(*p));
    if (!p) {
      return -1;
    }
    s->items = p;
    s->cap = cap;
  }
  s->items[s->len++] = *f;
  return 0;
}

static inline bool is_free(int8_t v, int8_t thr)
{
  return v >= 0 && v < thr;
}

static bool is_frontier(const tb4_grid *g, uint32_t x, uint32_t y, int8_t thr)
{
  if (!is_free(tb4_grid_get(g, x, y), thr)) {
    return false;
  }
  static const int d[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (int k = 0; k < 4; k++) {
    int64_t nx = (int64_t)x + d[k][0], ny = (int64_t)y + d[k][1];
    if (tb4_grid_in_bounds(g, nx, ny) && tb4_grid_get(g, (uint32_t)nx, (uint32_t)ny) == TB4_UNKNOWN) {
      return true;
    }
  }
  return false;
}

static double dist2(double ax, double ay, double bx, double by)
{
  return (ax - bx) * (ax - bx) + (ay - by) * (ay - by);
}

static int by_score_desc(const void *a, const void *b)
{
  double sa = ((const tb4_frontier *)a)->score, sb = ((const tb4_frontier *)b)->score;
  return (sa < sb) - (sa > sb);
}

long tb4_find_frontiers(const tb4_grid *g, double robot_x, double robot_y,
                        const tb4_explore_params *p, tb4_frontier_set *out)
{
  out->len = 0;
  if (!tb4_grid_valid(g)) {
    return 0;
  }
  const uint32_t w = g->meta.width, h = g->meta.height;
  const size_t n = (size_t)w * h;
  if (reserve_scratch(out, n) != 0) {
    return -1;
  }
  for (uint32_t y = 0; y < h; y++) {
    for (uint32_t x = 0; x < w; x++) {
      out->flag[tb4_grid_index(g, x, y)] = is_frontier(g, x, y, p->free_threshold);
    }
  }
  memset(out->seen, 0, n);

  for (size_t start = 0; start < n; start++) {
    if (!out->flag[start] || out->seen[start]) {
      continue;
    }
    size_t qhead = 0, qtail = 0, nm = 0;
    out->seen[start] = 1;
    out->queue[qtail++] = (uint32_t)start;
    while (qhead < qtail) {
      uint32_t i = out->queue[qhead++];
      out->members[nm++] = i;
      int64_t x = i % w, y = i / w;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (!tb4_grid_in_bounds(g, x + dx, y + dy)) {
            continue;
          }
          size_t j = tb4_grid_index(g, (uint32_t)(x + dx), (uint32_t)(y + dy));
          if (out->flag[j] && !out->seen[j]) {
            out->seen[j] = 1;
            out->queue[qtail++] = (uint32_t)j;
          }
        }
      }
    }
    if (nm < p->min_frontier_cells) {
      continue;
    }
    double sx = 0.0, sy = 0.0;
    for (size_t k = 0; k < nm; k++) {
      double wx, wy;
      tb4_grid_cell_to_world(g, out->members[k] % w, out->members[k] / w, &wx, &wy);
      sx += wx;
      sy += wy;
    }
    tb4_frontier f = {0};
    f.cells = nm;
    f.cx = sx / (double)nm;
    f.cy = sy / (double)nm;
    double best = INFINITY;
    for (size_t k = 0; k < nm; k++) {
      double wx, wy;
      tb4_grid_cell_to_world(g, out->members[k] % w, out->members[k] / w, &wx, &wy);
      double d = dist2(wx, wy, f.cx, f.cy);
      if (d < best) {
        best = d;
        f.gx = wx;
        f.gy = wy;
      }
    }
    f.size_m = (double)nm * g->meta.resolution;
    f.distance = sqrt(dist2(f.gx, f.gy, robot_x, robot_y));
    f.score = p->size_weight * f.size_m - p->distance_weight * f.distance;
    if (push_frontier(out, &f) != 0) {
      return -1;
    }
  }
  qsort(out->items, out->len, sizeof(tb4_frontier), by_score_desc);
  return (long)out->len;
}

const tb4_frontier *tb4_select_goal(const tb4_frontier_set *s, const double (*blacklist)[2],
                                    size_t n, const tb4_explore_params *p)
{
  const double r2 = p->blacklist_radius * p->blacklist_radius;
  for (size_t i = 0; i < s->len; i++) {
    const tb4_frontier *f = &s->items[i];
    if (f->distance < p->min_goal_distance) {
      continue;
    }
    bool ok = true;
    for (size_t k = 0; k < n && ok; k++) {
      ok = dist2(blacklist[k][0], blacklist[k][1], f->gx, f->gy) > r2;
    }
    if (ok) {
      return f;
    }
  }
  return NULL;
}
