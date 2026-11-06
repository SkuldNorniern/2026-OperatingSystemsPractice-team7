#ifndef TB4_MARKERS_H
#define TB4_MARKERS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tb4/grid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int64_t id;
  double camera[3];
  double bearing[3];
  double pnp_range;
} tb4_observation;

typedef enum { TB4_SRC_RAYCAST = 0, TB4_SRC_PNP = 1 } tb4_source;

typedef struct {
  double x, y;
  tb4_source source;
} tb4_estimate;

typedef struct {
  double max_range;
  int8_t occupied_threshold;
  double range_ratio_gate;
  bool allow_pnp_fallback;
  double max_abs_bearing_z;
} tb4_locate_params;

tb4_locate_params tb4_locate_default_params(void);

bool tb4_locate(const tb4_grid *grid, const tb4_observation *o, const tb4_locate_params *p,
                tb4_estimate *out);

typedef struct {
  size_t min_views;
  double max_spread;
  size_t max_history;
} tb4_track_params;

tb4_track_params tb4_track_default_params(void);

typedef struct {
  int64_t id;
  double x, y;
  size_t views;
  double spread;
  bool confirmed;
} tb4_target;

#define TB4_MAX_TRACKED_IDS 64

typedef struct {
  int64_t id;
  double *xs, *ys;
  size_t len, next;
} tb4_track;

typedef struct {
  tb4_track_params params;
  tb4_track tracks[TB4_MAX_TRACKED_IDS];
  size_t n;
  double *scratch;
} tb4_tracker;

int  tb4_tracker_init(tb4_tracker *t, tb4_track_params p);
void tb4_tracker_free(tb4_tracker *t);
int  tb4_tracker_add(tb4_tracker *t, int64_t id, const tb4_estimate *e);
bool tb4_tracker_target(tb4_tracker *t, int64_t id, tb4_target *out);
size_t tb4_tracker_targets(tb4_tracker *t, tb4_target *out, size_t max);

int tb4_targets_csv(const tb4_target *targets, size_t n, char *buf, size_t cap);

#ifdef __cplusplus
}
#endif
#endif
