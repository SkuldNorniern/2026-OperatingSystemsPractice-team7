#ifndef TB4_EXPLORE_H
#define TB4_EXPLORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tb4/grid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int8_t free_threshold;
  size_t min_frontier_cells;
  double size_weight;
  double distance_weight;
  double blacklist_radius;
  double min_goal_distance;
} tb4_explore_params;

tb4_explore_params tb4_explore_default_params(void);

typedef struct {
  size_t cells;
  double size_m;
  double cx, cy;
  double gx, gy;
  double distance;
  double score;
} tb4_frontier;

typedef struct {
  tb4_frontier *items;
  size_t len, cap;
  uint8_t *flag;
  uint8_t *seen;
  uint32_t *queue;
  uint32_t *members;
  size_t scratch_cells;
} tb4_frontier_set;

void tb4_frontier_set_init(tb4_frontier_set *s);
void tb4_frontier_set_free(tb4_frontier_set *s);

long tb4_find_frontiers(const tb4_grid *g, double robot_x, double robot_y,
                        const tb4_explore_params *p, tb4_frontier_set *out);

const tb4_frontier *tb4_select_goal(const tb4_frontier_set *s, const double (*blacklist)[2],
                                    size_t n, const tb4_explore_params *p);

#ifdef __cplusplus
}
#endif
#endif
