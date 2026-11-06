#include "tb4/grid.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void tb4_grid_init_empty(tb4_grid *g)
{
  memset(g, 0, sizeof(*g));
}

static int reserve(tb4_grid *g, size_t n)
{
  if (n <= g->cap) {
    return 0;
  }
  int8_t *p = realloc(g->data, n);
  if (!p) {
    return -1;
  }
  g->data = p;
  g->cap = n;
  return 0;
}

int tb4_grid_init(tb4_grid *g, tb4_grid_meta meta, int8_t fill)
{
  tb4_grid_init_empty(g);
  size_t n = (size_t)meta.width * meta.height;
  if (reserve(g, n) != 0) {
    return -1;
  }
  g->meta = meta;
  memset(g->data, fill, n);
  return 0;
}

int tb4_grid_assign(tb4_grid *g, tb4_grid_meta meta, const int8_t *data)
{
  size_t n = (size_t)meta.width * meta.height;
  if (reserve(g, n) != 0) {
    return -1;
  }
  g->meta = meta;
  if (n) {
    memcpy(g->data, data, n);
  }
  return 0;
}

void tb4_grid_free(tb4_grid *g)
{
  free(g->data);
  tb4_grid_init_empty(g);
}

bool tb4_grid_valid(const tb4_grid *g)
{
  return g->data && g->meta.width && g->meta.height && g->meta.resolution > 0.0;
}

void tb4_grid_world_to_cell_signed(const tb4_grid *g, double x, double y, int64_t *cx, int64_t *cy)
{
  *cx = (int64_t)floor((x - g->meta.origin_x) / g->meta.resolution);
  *cy = (int64_t)floor((y - g->meta.origin_y) / g->meta.resolution);
}

bool tb4_grid_world_to_cell(const tb4_grid *g, double x, double y, uint32_t *cx, uint32_t *cy)
{
  int64_t sx, sy;
  tb4_grid_world_to_cell_signed(g, x, y, &sx, &sy);
  if (!tb4_grid_in_bounds(g, sx, sy)) {
    return false;
  }
  *cx = (uint32_t)sx;
  *cy = (uint32_t)sy;
  return true;
}

void tb4_grid_cell_to_world(const tb4_grid *g, uint32_t cx, uint32_t cy, double *x, double *y)
{
  *x = g->meta.origin_x + ((double)cx + 0.5) * g->meta.resolution;
  *y = g->meta.origin_y + ((double)cy + 0.5) * g->meta.resolution;
}

bool tb4_grid_raycast(const tb4_grid *g, double x, double y, double dx, double dy,
                      double max_range, int8_t occupied_threshold, tb4_ray_hit *hit)
{
  double norm = hypot(dx, dy);
  if (norm < 1e-12 || !tb4_grid_valid(g)) {
    return false;
  }
  dx /= norm;
  dy /= norm;
  const double res = g->meta.resolution;
  int64_t cx, cy;
  tb4_grid_world_to_cell_signed(g, x, y, &cx, &cy);

  const int step_x = dx > 0.0 ? 1 : -1;
  const int step_y = dy > 0.0 ? 1 : -1;
  double bx = g->meta.origin_x + (double)(cx + (step_x > 0)) * res;
  double by = g->meta.origin_y + (double)(cy + (step_y > 0)) * res;
  double t_max_x = fabs(dx) > 1e-12 ? (bx - x) / dx : INFINITY;
  double t_max_y = fabs(dy) > 1e-12 ? (by - y) / dy : INFINITY;
  const double t_delta_x = fabs(dx) > 1e-12 ? res / fabs(dx) : INFINITY;
  const double t_delta_y = fabs(dy) > 1e-12 ? res / fabs(dy) : INFINITY;

  double t = 0.0;
  for (;;) {
    if (t > max_range || !tb4_grid_in_bounds(g, cx, cy)) {
      return false;
    }
    if (tb4_grid_get(g, (uint32_t)cx, (uint32_t)cy) >= occupied_threshold) {
      hit->x = x + dx * t;
      hit->y = y + dy * t;
      hit->range = t;
      return true;
    }
    if (t_max_x < t_max_y) {
      t = t_max_x;
      t_max_x += t_delta_x;
      cx += step_x;
    } else {
      t = t_max_y;
      t_max_y += t_delta_y;
      cy += step_y;
    }
  }
}
