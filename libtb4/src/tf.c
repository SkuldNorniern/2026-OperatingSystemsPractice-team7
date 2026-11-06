#include "tb4/tf.h"

#include <stdlib.h>
#include <string.h>

#define MAX_CHAIN 64
#define EDGE_CAP 1024

typedef struct {
  int64_t stamp;
  tb4_iso tf;
} sample;

typedef struct {
  char child[TB4_TF_NAME_MAX];
  char parent[TB4_TF_NAME_MAX];
  bool is_static;
  size_t len;
  sample *samples;
} edge;

struct tb4_tf_buffer {
  edge *edges;
  size_t n_edges, cap_edges;
  int64_t cache_ns, tolerance_ns;
};

static const char *strip(const char *f)
{
  return f[0] == '/' ? f + 1 : f;
}

tb4_tf_buffer *tb4_tf_create(double cache_seconds, double tolerance_seconds)
{
  tb4_tf_buffer *b = calloc(1, sizeof(*b));
  if (!b) {
    return NULL;
  }
  b->cache_ns = (int64_t)(cache_seconds * 1e9);
  b->tolerance_ns = (int64_t)(tolerance_seconds * 1e9);
  return b;
}

void tb4_tf_destroy(tb4_tf_buffer *b)
{
  if (!b) {
    return;
  }
  for (size_t i = 0; i < b->n_edges; i++) {
    free(b->edges[i].samples);
  }
  free(b->edges);
  free(b);
}

static edge *find_edge(const tb4_tf_buffer *b, const char *child)
{
  for (size_t i = 0; i < b->n_edges; i++) {
    if (strcmp(b->edges[i].child, child) == 0) {
      return &b->edges[i];
    }
  }
  return NULL;
}

static void copy_name(char *dst, const char *src)
{
  strncpy(dst, src, TB4_TF_NAME_MAX - 1);
  dst[TB4_TF_NAME_MAX - 1] = '\0';
}

int tb4_tf_insert(tb4_tf_buffer *b, const char *parent, const char *child, int64_t stamp,
                  tb4_iso tf, bool is_static)
{
  parent = strip(parent);
  child = strip(child);
  edge *e = find_edge(b, child);
  if (!e) {
    if (b->n_edges == b->cap_edges) {
      size_t cap = b->cap_edges ? b->cap_edges * 2 : 16;
      edge *p = realloc(b->edges, cap * sizeof(*p));
      if (!p) {
        return -1;
      }
      b->edges = p;
      b->cap_edges = cap;
    }
    e = &b->edges[b->n_edges];
    memset(e, 0, sizeof(*e));
    e->samples = malloc(EDGE_CAP * sizeof(sample));
    if (!e->samples) {
      return -1;
    }
    b->n_edges++;
    copy_name(e->child, child);
    copy_name(e->parent, parent);
    e->is_static = is_static;
  }
  if (strncmp(e->parent, parent, TB4_TF_NAME_MAX - 1) != 0 || e->is_static != is_static) {
    copy_name(e->parent, parent);
    e->is_static = is_static;
    e->len = 0;
  }
  if (is_static) {
    e->samples[0].stamp = stamp;
    e->samples[0].tf = tf;
    e->len = 1;
    return 0;
  }
  size_t pos = e->len;
  while (pos > 0 && e->samples[pos - 1].stamp > stamp) {
    pos--;
  }
  if (pos > 0 && e->samples[pos - 1].stamp == stamp) {
    e->samples[pos - 1].tf = tf;
    return 0;
  }
  if (e->len == EDGE_CAP) {
    if (pos == 0) {
      return 0;
    }
    memmove(&e->samples[0], &e->samples[1], (EDGE_CAP - 1) * sizeof(sample));
    e->len--;
    pos--;
  }
  memmove(&e->samples[pos + 1], &e->samples[pos], (e->len - pos) * sizeof(sample));
  e->samples[pos].stamp = stamp;
  e->samples[pos].tf = tf;
  e->len++;
  int64_t newest = e->samples[e->len - 1].stamp;
  size_t drop = 0;
  while (drop < e->len && newest - e->samples[drop].stamp > b->cache_ns) {
    drop++;
  }
  if (drop) {
    memmove(&e->samples[0], &e->samples[drop], (e->len - drop) * sizeof(sample));
    e->len -= drop;
  }
  return 0;
}

bool tb4_tf_has_frame(const tb4_tf_buffer *b, const char *frame)
{
  frame = strip(frame);
  for (size_t i = 0; i < b->n_edges; i++) {
    if (strcmp(b->edges[i].child, frame) == 0 || strcmp(b->edges[i].parent, frame) == 0) {
      return true;
    }
  }
  return false;
}

static tb4_tf_status edge_at(const tb4_tf_buffer *b, const edge *e, int64_t stamp, tb4_iso *out)
{
  const sample *s = e->samples;
  int64_t oldest = s[0].stamp, newest = s[e->len - 1].stamp;
  if (e->is_static || stamp == TB4_TF_LATEST) {
    *out = s[e->len - 1].tf;
    return TB4_TF_OK;
  }
  if (stamp <= oldest) {
    if (oldest - stamp > b->tolerance_ns) {
      return TB4_TF_EXTRAPOLATION;
    }
    *out = s[0].tf;
    return TB4_TF_OK;
  }
  if (stamp >= newest) {
    if (stamp - newest > b->tolerance_ns) {
      return TB4_TF_EXTRAPOLATION;
    }
    *out = s[e->len - 1].tf;
    return TB4_TF_OK;
  }
  size_t lo = 0, hi = e->len - 1;
  while (lo < hi) {
    size_t mid = (lo + hi) / 2;
    if (s[mid].stamp < stamp) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  const sample *a = &s[lo - 1], *c = &s[lo];
  double alpha = (double)(stamp - a->stamp) / (double)(c->stamp - a->stamp);
  *out = tb4_iso_interp(a->tf, c->tf, alpha);
  return TB4_TF_OK;
}

static size_t chain(const tb4_tf_buffer *b, const char *frame, const edge **out)
{
  size_t n = 0;
  const char *cur = frame;
  const edge *e;
  while (n < MAX_CHAIN && (e = find_edge(b, cur)) != NULL) {
    out[n++] = e;
    cur = e->parent;
  }
  return n;
}

static bool on_chain(const char *f, const char *start, const edge **c, size_t n)
{
  if (strcmp(f, start) == 0) {
    return true;
  }
  for (size_t i = 0; i < n; i++) {
    if (strcmp(c[i]->parent, f) == 0) {
      return true;
    }
  }
  return false;
}

tb4_tf_status tb4_tf_lookup(const tb4_tf_buffer *b, const char *target, const char *source,
                            int64_t stamp, tb4_iso *out)
{
  target = strip(target);
  source = strip(source);
  if (strcmp(target, source) == 0) {
    *out = tb4_iso_identity();
    return TB4_TF_OK;
  }
  if (!tb4_tf_has_frame(b, target) || !tb4_tf_has_frame(b, source)) {
    return TB4_TF_UNKNOWN_FRAME;
  }
  const edge *tc[MAX_CHAIN], *sc[MAX_CHAIN];
  size_t nt = chain(b, target, tc), ns = chain(b, source, sc);
  const char *troot = nt ? tc[nt - 1]->parent : target;
  const char *sroot = ns ? sc[ns - 1]->parent : source;
  if (strcmp(troot, sroot) != 0) {
    return TB4_TF_NOT_CONNECTED;
  }
  const char *lca = source;
  size_t up = 0;
  if (!on_chain(source, target, tc, nt)) {
    for (; up < ns; up++) {
      if (on_chain(sc[up]->parent, target, tc, nt)) {
        lca = sc[up]->parent;
        up++;
        break;
      }
    }
  }
  tb4_iso lca_t_source = tb4_iso_identity();
  for (size_t i = up; i-- > 0;) {
    tb4_iso e;
    tb4_tf_status st = edge_at(b, sc[i], stamp, &e);
    if (st != TB4_TF_OK) {
      return st;
    }
    lca_t_source = tb4_iso_mul(lca_t_source, e);
  }
  size_t down = 0;
  while (down < nt && strcmp(tc[down]->child, lca) != 0) {
    down++;
  }
  tb4_iso lca_t_target = tb4_iso_identity();
  for (size_t i = down; i-- > 0;) {
    tb4_iso e;
    tb4_tf_status st = edge_at(b, tc[i], stamp, &e);
    if (st != TB4_TF_OK) {
      return st;
    }
    lca_t_target = tb4_iso_mul(lca_t_target, e);
  }
  *out = tb4_iso_mul(tb4_iso_inv(lca_t_target), lca_t_source);
  return TB4_TF_OK;
}

const char *tb4_tf_status_str(tb4_tf_status s)
{
  switch (s) {
    case TB4_TF_OK: return "ok";
    case TB4_TF_UNKNOWN_FRAME: return "unknown frame";
    case TB4_TF_NOT_CONNECTED: return "frames not connected";
    case TB4_TF_EXTRAPOLATION: return "extrapolation (stamp outside buffer)";
  }
  return "?";
}
