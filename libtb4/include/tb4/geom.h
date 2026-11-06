#ifndef TB4_GEOM_H
#define TB4_GEOM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { double x, y, z; } tb4_vec3;
typedef struct { double x, y, z, w; } tb4_quat;
typedef struct { tb4_vec3 t; tb4_quat q; } tb4_iso;

tb4_iso  tb4_iso_identity(void);
tb4_iso  tb4_iso_make(double x, double y, double z, tb4_quat q);
tb4_iso  tb4_iso_mul(tb4_iso a, tb4_iso b);
tb4_iso  tb4_iso_inv(tb4_iso a);
tb4_vec3 tb4_iso_apply(tb4_iso a, tb4_vec3 p);
tb4_iso  tb4_iso_interp(tb4_iso a, tb4_iso b, double alpha);

tb4_quat tb4_quat_mul(tb4_quat a, tb4_quat b);
tb4_quat tb4_quat_normalize(tb4_quat q);
tb4_vec3 tb4_quat_rotate(tb4_quat q, tb4_vec3 v);
tb4_quat tb4_quat_from_rpy(double roll, double pitch, double yaw);
tb4_quat tb4_quat_from_yaw(double yaw);
double   tb4_quat_yaw(tb4_quat q);

double   tb4_vec3_norm(tb4_vec3 v);

#ifdef __cplusplus
}
#endif
#endif
