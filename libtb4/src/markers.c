#include "tb4/markers.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

tb4_locate_params tb4_locate_default_params(void)
{
  tb4_locate_params p = {
    .max_range = 4.0,
    .occupied_threshold = 65,
    .range_ratio_gate = 3.0,
    .allow_pnp_fallback = true,
    .max_abs_bearing_z = 0.8,
  };
  return p;
}

tb4_track_params tb4_track_default_params(void)
{
  tb4_track_params p = {.min_views = 5, .max_spread = 0.25, .max_history = 200};
  return p;
}

/* raycast bearing through map, so marker size doesnt matter. pnp range is fallback */
bool tb4_locate(const tb4_grid *grid, const tb4_observation *o, const tb4_locate_params *p,
                tb4_estimate *out)
{
  double bx = o->bearing[0], by = o->bearing[1], bz = o->bearing[2];
  if (fabs(bz) > p->max_abs_bearing_z) {
    return false;
  }
  double horiz = hypot(bx, by);
  if (horiz < 1e-6) {
    return false;
  }
  double dx = bx / horiz, dy = by / horiz;
  double pnp_ground = o->pnp_range * horiz;

  tb4_ray_hit hit;
  if (grid && tb4_grid_raycast(grid, o->camera[0], o->camera[1], dx, dy, p->max_range,
                               p->occupied_threshold, &hit)) {
    double ratio = hit.range > 1e-6 ? pnp_ground / hit.range : INFINITY;
    if (isfinite(ratio) && ratio <= p->range_ratio_gate && ratio >= 1.0 / p->range_ratio_gate) {
      out->x = hit.x;
      out->y = hit.y;
      out->source = TB4_SRC_RAYCAST;
      return true;
    }
  }
  if (p->allow_pnp_fallback && pnp_ground > 0.0 && pnp_ground <= p->max_range) {
    out->x = o->camera[0] + dx * pnp_ground;
    out->y = o->camera[1] + dy * pnp_ground;
    out->source = TB4_SRC_PNP;
    return true;
  }
  return false;
}

int tb4_tracker_init(tb4_tracker *t, tb4_track_params p)
{
  memset(t, 0, sizeof(*t));
  if (p.max_history == 0) {
    p.max_history = 1;
  }
  t->params = p;
  t->scratch = malloc(p.max_history * sizeof(double));
  return t->scratch ? 0 : -1;
}

void tb4_tracker_free(tb4_tracker *t)
{
  for (size_t i = 0; i < t->n; i++) {
    free(t->tracks[i].xs);
    free(t->tracks[i].ys);
  }
  free(t->scratch);
  memset(t, 0, sizeof(*t));
}

static tb4_track *find(tb4_tracker *t, int64_t id)
{
  for (size_t i = 0; i < t->n; i++) {
    if (t->tracks[i].id == id) {
      return &t->tracks[i];
    }
  }
  return NULL;
}

int tb4_tracker_add(tb4_tracker *t, int64_t id, const tb4_estimate *e)
{
  tb4_track *tr = find(t, id);
  if (!tr) {
    if (t->n == TB4_MAX_TRACKED_IDS) {
      return -1;
    }
    size_t pos = 0;
    while (pos < t->n && t->tracks[pos].id < id) {
      pos++;
    }
    double *xs = malloc(t->params.max_history * sizeof(double));
    double *ys = malloc(t->params.max_history * sizeof(double));
    if (!xs || !ys) {
      free(xs);
      free(ys);
      return -1;
    }
    memmove(&t->tracks[pos + 1], &t->tracks[pos], (t->n - pos) * sizeof(tb4_track));
    tr = &t->tracks[pos];
    tr->id = id;
    tr->xs = xs;
    tr->ys = ys;
    tr->len = 0;
    tr->next = 0;
    t->n++;
  }
  tr->xs[tr->next] = e->x;
  tr->ys[tr->next] = e->y;
  tr->next = (tr->next + 1) % t->params.max_history;
  if (tr->len < t->params.max_history) {
    tr->len++;
  }
  return 0;
}

static int cmp_double(const void *a, const void *b)
{
  double x = *(const double *)a, y = *(const double *)b;
  return (x > y) - (x < y);
}

static double median(double *scratch, const double *v, size_t n)
{
  memcpy(scratch, v, n * sizeof(double));
  qsort(scratch, n, sizeof(double), cmp_double);
  return n % 2 ? scratch[n / 2] : 0.5 * (scratch[n / 2 - 1] + scratch[n / 2]);
}

static void summarize(tb4_tracker *t, const tb4_track *tr, tb4_target *out)
{
  size_t n = tr->len;
  double mx = median(t->scratch, tr->xs, n);
  double my = median(t->scratch, tr->ys, n);
  for (size_t i = 0; i < n; i++) {
    t->scratch[i] = hypot(tr->xs[i] - mx, tr->ys[i] - my);
  }
  qsort(t->scratch, n, sizeof(double), cmp_double);
  double spread = n % 2 ? t->scratch[n / 2] : 0.5 * (t->scratch[n / 2 - 1] + t->scratch[n / 2]);
  out->id = tr->id;
  out->x = mx;
  out->y = my;
  out->views = n;
  out->spread = spread;
  out->confirmed = n >= t->params.min_views && spread <= t->params.max_spread;
}

bool tb4_tracker_target(tb4_tracker *t, int64_t id, tb4_target *out)
{
  tb4_track *tr = find(t, id);
  if (!tr || tr->len == 0) {
    return false;
  }
  summarize(t, tr, out);
  return true;
}

size_t tb4_tracker_targets(tb4_tracker *t, tb4_target *out, size_t max)
{
  size_t k = 0;
  for (size_t i = 0; i < t->n && k < max; i++) {
    if (t->tracks[i].len) {
      summarize(t, &t->tracks[i], &out[k++]);
    }
  }
  return k;
}

int tb4_targets_csv(const tb4_target *targets, size_t n, char *buf, size_t cap)
{
  int total = snprintf(buf, cap, "id,x,y\n");
  for (size_t i = 0; i < n; i++) {
    size_t off = (size_t)total < cap ? (size_t)total : cap;
    total += snprintf(buf + off, cap - off, "%" PRId64 ",%.3f,%.3f\n", targets[i].id,
                      targets[i].x, targets[i].y);
  }
  return total;
}
