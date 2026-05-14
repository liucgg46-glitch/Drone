#ifndef __QUATERNION_H
#define __QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quat_t;

typedef struct {
    float roll;   /* deg */
    float pitch;  /* deg */
    float yaw;    /* deg */
} euler_t;

void  quat_set_identity(quat_t *q);
void  quat_normalize(quat_t *q);
quat_t quat_multiply(quat_t a, quat_t b);
euler_t quat_to_euler_deg(quat_t q);

#ifdef __cplusplus
}
#endif

#endif
