#ifndef TB4_GRID_H
#define TB4_GRID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TB4_UNKNOWN ((int8_t)-1)

typedef struct {
  uint32_t width, height;
  double resolution;
  double origin_x, origin_y;
} tb4_grid_meta;

typedef struct {
  tb4_grid_meta meta;
  int8_t *data;
  size_t cap;
} tb4_grid;

typedef struct {
  double x, y, range;
} tb4_ray_hit;

void tb4_grid_init_empty(tb4_grid *g);
int  tb4_grid_init(tb4_grid *g, tb4_grid_meta meta, int8_t fill);
int  tb4_grid_assign(tb4_grid *g, tb4_grid_meta meta, const int8_t *data);
void tb4_grid_free(tb4_grid *g);
bool tb4_grid_valid(const tb4_grid *g);

static inline size_t tb4_grid_index(const tb4_grid *g, uint32_t cx, uint32_t cy)
{
  return (size_t)cy * g->meta.width + cx;
}
static inline int8_t tb4_grid_get(const tb4_grid *g, uint32_t cx, uint32_t cy)
{
  return g->data[tb4_grid_index(g, cx, cy)];
}
static inline void tb4_grid_set(tb4_grid *g, uint32_t cx, uint32_t cy, int8_t v)
{
  g->data[tb4_grid_index(g, cx, cy)] = v;
}
static inline bool tb4_grid_in_bounds(const tb4_grid *g, int64_t cx, int64_t cy)
{
  return cx >= 0 && cy >= 0 && cx < (int64_t)g->meta.width && cy < (int64_t)g->meta.height;
}

void tb4_grid_world_to_cell_signed(const tb4_grid *g, double x, double y, int64_t *cx, int64_t *cy);
bool tb4_grid_world_to_cell(const tb4_grid *g, double x, double y, uint32_t *cx, uint32_t *cy);
void tb4_grid_cell_to_world(const tb4_grid *g, uint32_t cx, uint32_t cy, double *x, double *y);

bool tb4_grid_raycast(const tb4_grid *g, double x, double y, double dx, double dy,
                      double max_range, int8_t occupied_threshold, tb4_ray_hit *hit);

#ifdef __cplusplus
}
#endif
#endif
