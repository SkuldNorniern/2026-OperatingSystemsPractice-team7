#define _GNU_SOURCE
#include "common.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <rosidl_runtime_c/string_functions.h>

void t7_params_init(t7_params *p, rclc_support_t *support, const rcl_node_t *node)
{
  memset(p, 0, sizeof(*p));
  if (rcl_arguments_get_param_overrides(&support->context.global_arguments, &p->ov) != RCL_RET_OK) {
    rcl_reset_error();
    p->ov = NULL;
  }
  const char *fq = rcl_node_get_fully_qualified_name(node);
  const char *bare = rcl_node_get_name(node);
  snprintf(p->names[0], sizeof(p->names[0]), "%s", fq ? fq : "");
  snprintf(p->names[1], sizeof(p->names[1]), "%s", bare ? bare : "");
  snprintf(p->names[2], sizeof(p->names[2]), "/**");
}

void t7_params_fini(t7_params *p)
{
  if (p->ov) {
    rcl_yaml_node_struct_fini(p->ov);
  }
  p->ov = NULL;
}

static bool has_value(const rcl_variant_t *v)
{
  return v->bool_value || v->integer_value || v->double_value || v->string_value ||
         v->byte_array_value || v->bool_array_value || v->integer_array_value ||
         v->double_array_value || v->string_array_value;
}

/* later entries win like rclcpp. rcl_yaml_node_struct_get not used, it inserts missing entries */
static const rcl_variant_t *lookup(const t7_params *p, const char *name)
{
  if (!p->ov) {
    return NULL;
  }
  const rcl_variant_t *found = NULL;
  for (size_t i = 0; i < p->ov->num_nodes; i++) {
    const char *n = p->ov->node_names[i];
    if (strcmp(n, p->names[0]) != 0 && strcmp(n, p->names[1]) != 0 && strcmp(n, p->names[2]) != 0) {
      continue;
    }
    const rcl_node_params_t *np = &p->ov->params[i];
    for (size_t j = 0; j < np->num_params; j++) {
      if (strcmp(np->parameter_names[j], name) == 0 && has_value(&np->parameter_values[j])) {
        found = &np->parameter_values[j];
      }
    }
  }
  return found;
}

double t7_param_double(t7_params *p, const char *name, double def)
{
  const rcl_variant_t *v = lookup(p, name);
  if (v && v->double_value) return *v->double_value;
  if (v && v->integer_value) return (double)*v->integer_value;
  return def;
}

int64_t t7_param_int(t7_params *p, const char *name, int64_t def)
{
  const rcl_variant_t *v = lookup(p, name);
  return v && v->integer_value ? *v->integer_value : def;
}

bool t7_param_bool(t7_params *p, const char *name, bool def)
{
  const rcl_variant_t *v = lookup(p, name);
  return v && v->bool_value ? *v->bool_value : def;
}

const char *t7_param_string(t7_params *p, const char *name, const char *def, char *out, size_t cap)
{
  const rcl_variant_t *v = lookup(p, name);
  snprintf(out, cap, "%s", v && v->string_value ? v->string_value : def);
  return out;
}

size_t t7_param_int_array(t7_params *p, const char *name, int64_t *out, size_t max)
{
  const rcl_variant_t *v = lookup(p, name);
  if (!v || !v->integer_array_value) {
    return 0;
  }
  size_t n = v->integer_array_value->size < max ? v->integer_array_value->size : max;
  memcpy(out, v->integer_array_value->values, n * sizeof(int64_t));
  return n;
}

rmw_qos_profile_t t7_qos(size_t depth, bool reliable, bool transient_local)
{
  rmw_qos_profile_t q = rmw_qos_profile_default;
  q.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  q.depth = depth;
  q.reliability = reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  q.durability = transient_local ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL : RMW_QOS_POLICY_DURABILITY_VOLATILE;
  return q;
}

static void tf_insert_all(tb4_tf_buffer *buf, const tf2_msgs__msg__TFMessage *m, bool is_static)
{
  for (size_t i = 0; i < m->transforms.size; i++) {
    const geometry_msgs__msg__TransformStamped *t = &m->transforms.data[i];
    tb4_tf_insert(buf, t->header.frame_id.data, t->child_frame_id.data, t7_stamp(&t->header.stamp),
                  t7_iso_from_transform(&t->transform), is_static);
  }
}

static void tf_dyn_cb(const void *msg, void *ctx)
{
  tf_insert_all(((t7_tf_listener *)ctx)->buf, msg, false);
}

static void tf_static_cb(const void *msg, void *ctx)
{
  tf_insert_all(((t7_tf_listener *)ctx)->buf, msg, true);
}

void t7_tf_listener_init(t7_tf_listener *l, rcl_node_t *node, rclc_executor_t *ex,
                         const char *tf_topic, const char *tf_static_topic,
                         double cache_s, double tolerance_s)
{
  l->buf = tb4_tf_create(cache_s, tolerance_s);
  if (!l->buf) {
    RCUTILS_LOG_FATAL("out of memory");
    exit(1);
  }
  const rosidl_message_type_support_t *ts = ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage);
  rmw_qos_profile_t qd = t7_qos(100, true, false);
  rmw_qos_profile_t qs = t7_qos(100, true, true);
  l->sub_dyn = rcl_get_zero_initialized_subscription();
  l->sub_static = rcl_get_zero_initialized_subscription();
  T7_CHECK(rclc_subscription_init(&l->sub_dyn, node, ts, tf_topic, &qd));
  T7_CHECK(rclc_subscription_init(&l->sub_static, node, ts, tf_static_topic, &qs));
  tf2_msgs__msg__TFMessage__init(&l->msg_dyn);
  tf2_msgs__msg__TFMessage__init(&l->msg_static);
  T7_CHECK(rclc_executor_add_subscription_with_context(ex, &l->sub_dyn, &l->msg_dyn, tf_dyn_cb, l, ON_NEW_DATA));
  T7_CHECK(rclc_executor_add_subscription_with_context(ex, &l->sub_static, &l->msg_static, tf_static_cb, l, ON_NEW_DATA));
}

void t7_tf_listener_fini(t7_tf_listener *l, rcl_node_t *node)
{
  T7_IGNORE(rcl_subscription_fini(&l->sub_dyn, node));
  T7_IGNORE(rcl_subscription_fini(&l->sub_static, node));
  tf2_msgs__msg__TFMessage__fini(&l->msg_dyn);
  tf2_msgs__msg__TFMessage__fini(&l->msg_static);
  tb4_tf_destroy(l->buf);
}

tb4_iso t7_iso_from_transform(const geometry_msgs__msg__Transform *t)
{
  tb4_quat q = {t->rotation.x, t->rotation.y, t->rotation.z, t->rotation.w};
  return tb4_iso_make(t->translation.x, t->translation.y, t->translation.z, q);
}

tb4_iso t7_iso_from_pose(const geometry_msgs__msg__Pose *p)
{
  tb4_quat q = {p->orientation.x, p->orientation.y, p->orientation.z, p->orientation.w};
  return tb4_iso_make(p->position.x, p->position.y, p->position.z, q);
}

bool t7_grid_from_msg(tb4_grid *g, const nav_msgs__msg__OccupancyGrid *m)
{
  const nav_msgs__msg__MapMetaData *i = &m->info;
  if (i->width == 0 || i->height == 0 || m->data.size != (size_t)i->width * i->height) {
    return false;
  }
  tb4_grid_meta meta = {i->width, i->height, i->resolution, i->origin.position.x, i->origin.position.y};
  return tb4_grid_assign(g, meta, m->data.data) == 0;
}

static volatile sig_atomic_t g_interrupted = 0;

static void on_signal(int sig)
{
  (void)sig;
  g_interrupted = 1;
}

void t7_install_signal_handlers(void)
{
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = on_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
}

bool t7_interrupted(void)
{
  return g_interrupted != 0;
}

void t7_spin(rclc_executor_t *ex, const volatile bool *stop)
{
  while (!g_interrupted && !(stop && *stop)) {
    rcl_ret_t rc = rclc_executor_spin_some(ex, RCL_MS_TO_NS(10));
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
      rcl_reset_error();
    }
  }
}

void t7_expand_home(const char *in, char *out, size_t cap)
{
  const char *home = getenv("HOME");
  if (strncmp(in, "~/", 2) == 0 && home) {
    snprintf(out, cap, "%s/%s", home, in + 2);
  } else {
    snprintf(out, cap, "%s", in);
  }
}

int t7_mkdirs(const char *path)
{
  char tmp[4096];
  snprintf(tmp, sizeof(tmp), "%s", path);
  for (char *p = tmp + 1; *p; p++) {
    if (*p == '/') {
      *p = '\0';
      if (mkdir(tmp, 0755) != 0 && errno != EEXIST) {
        return -1;
      }
      *p = '/';
    }
  }
  return (mkdir(tmp, 0755) != 0 && errno != EEXIST) ? -1 : 0;
}

int t7_write_atomic(const char *path, const char *data, size_t len)
{
  char tmp[4096];
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  FILE *f = fopen(tmp, "w");
  if (!f) {
    return -1;
  }
  size_t w = fwrite(data, 1, len, f);
  if (fclose(f) != 0 || w != len) {
    unlink(tmp);
    return -1;
  }
  return rename(tmp, path);
}

void t7_marker_setup(visualization_msgs__msg__Marker *m, const char *frame, const char *ns,
                     int32_t id, int32_t type, int32_t action)
{
  rosidl_runtime_c__String__assign(&m->header.frame_id, frame);
  rosidl_runtime_c__String__assign(&m->ns, ns);
  m->id = id;
  m->type = type;
  m->action = action;
  m->pose.orientation.w = 1.0;
}

void t7_marker_color(visualization_msgs__msg__Marker *m, float r, float g, float b, float a)
{
  m->color.r = r;
  m->color.g = g;
  m->color.b = b;
  m->color.a = a;
}
