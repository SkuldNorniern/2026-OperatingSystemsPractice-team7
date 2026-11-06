#ifndef TB4_TF_H
#define TB4_TF_H

#include <stdbool.h>
#include <stdint.h>

#include "tb4/geom.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TB4_TF_NAME_MAX 64
#define TB4_TF_LATEST ((int64_t)0)

typedef enum {
  TB4_TF_OK = 0,
  TB4_TF_UNKNOWN_FRAME,
  TB4_TF_NOT_CONNECTED,
  TB4_TF_EXTRAPOLATION,
} tb4_tf_status;

typedef struct tb4_tf_buffer tb4_tf_buffer;

static inline int64_t tb4_stamp(int32_t sec, uint32_t nanosec)
{
  return (int64_t)sec * 1000000000LL + (int64_t)nanosec;
}

tb4_tf_buffer *tb4_tf_create(double cache_seconds, double tolerance_seconds);
void tb4_tf_destroy(tb4_tf_buffer *b);

int tb4_tf_insert(tb4_tf_buffer *b, const char *parent, const char *child, int64_t stamp,
                  tb4_iso tf, bool is_static);

bool tb4_tf_has_frame(const tb4_tf_buffer *b, const char *frame);

tb4_tf_status tb4_tf_lookup(const tb4_tf_buffer *b, const char *target, const char *source,
                            int64_t stamp, tb4_iso *out);

const char *tb4_tf_status_str(tb4_tf_status s);

#ifdef __cplusplus
}
#endif
#endif
