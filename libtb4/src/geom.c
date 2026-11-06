#include "tb4/geom.h"

#include <math.h>

tb4_iso tb4_iso_identity(void)
{
  tb4_iso r = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
  return r;
}

tb4_iso tb4_iso_make(double x, double y, double z, tb4_quat q)
{
  tb4_iso r = {{x, y, z}, tb4_quat_normalize(q)};
  return r;
}

tb4_quat tb4_quat_mul(tb4_quat a, tb4_quat b)
{
  tb4_quat r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}

tb4_quat tb4_quat_normalize(tb4_quat q)
{
  double n = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (n < 1e-12) {
    tb4_quat id = {0.0, 0.0, 0.0, 1.0};
    return id;
  }
  tb4_quat r = {q.x / n, q.y / n, q.z / n, q.w / n};
  return r;
}

static tb4_vec3 cross(tb4_vec3 a, tb4_vec3 b)
{
  tb4_vec3 r = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
  return r;
}

tb4_vec3 tb4_quat_rotate(tb4_quat q, tb4_vec3 v)
{
  tb4_vec3 u = {q.x, q.y, q.z};
  tb4_vec3 uv = cross(u, v);
  tb4_vec3 uuv = cross(u, uv);
  tb4_vec3 r = {
    v.x + 2.0 * (q.w * uv.x + uuv.x),
    v.y + 2.0 * (q.w * uv.y + uuv.y),
    v.z + 2.0 * (q.w * uv.z + uuv.z),
  };
  return r;
}

tb4_vec3 tb4_iso_apply(tb4_iso a, tb4_vec3 p)
{
  tb4_vec3 r = tb4_quat_rotate(a.q, p);
  r.x += a.t.x;
  r.y += a.t.y;
  r.z += a.t.z;
  return r;
}

tb4_iso tb4_iso_mul(tb4_iso a, tb4_iso b)
{
  tb4_iso r;
  r.t = tb4_iso_apply(a, b.t);
  r.q = tb4_quat_normalize(tb4_quat_mul(a.q, b.q));
  return r;
}

tb4_iso tb4_iso_inv(tb4_iso a)
{
  tb4_iso r;
  r.q.x = -a.q.x;
  r.q.y = -a.q.y;
  r.q.z = -a.q.z;
  r.q.w = a.q.w;
  tb4_vec3 t = tb4_quat_rotate(r.q, a.t);
  r.t.x = -t.x;
  r.t.y = -t.y;
  r.t.z = -t.z;
  return r;
}

tb4_iso tb4_iso_interp(tb4_iso a, tb4_iso b, double alpha)
{
  tb4_iso r;
  r.t.x = a.t.x + (b.t.x - a.t.x) * alpha;
  r.t.y = a.t.y + (b.t.y - a.t.y) * alpha;
  r.t.z = a.t.z + (b.t.z - a.t.z) * alpha;

  tb4_quat qa = a.q, qb = b.q;
  double dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
  if (dot < 0.0) {
    qb.x = -qb.x; qb.y = -qb.y; qb.z = -qb.z; qb.w = -qb.w;
    dot = -dot;
  }
  double wa, wb;
  if (dot > 0.9995) {
    wa = 1.0 - alpha;
    wb = alpha;
  } else {
    double theta = acos(dot);
    double s = sin(theta);
    wa = sin((1.0 - alpha) * theta) / s;
    wb = sin(alpha * theta) / s;
  }
  tb4_quat q = {wa * qa.x + wb * qb.x, wa * qa.y + wb * qb.y, wa * qa.z + wb * qb.z, wa * qa.w + wb * qb.w};
  r.q = tb4_quat_normalize(q);
  return r;
}

tb4_quat tb4_quat_from_rpy(double roll, double pitch, double yaw)
{
  double cr = cos(roll * 0.5), sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
  tb4_quat q = {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  };
  return q;
}

tb4_quat tb4_quat_from_yaw(double yaw)
{
  tb4_quat q = {0.0, 0.0, sin(yaw * 0.5), cos(yaw * 0.5)};
  return q;
}

double tb4_quat_yaw(tb4_quat q)
{
  return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double tb4_vec3_norm(tb4_vec3 v)
{
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
