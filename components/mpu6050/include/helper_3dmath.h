// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D
// math helper 6/5/2012 by Jeff Rowberg <jeff@rowberg.net> Updates should
// (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <fastmath.h>

typedef struct {
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

typedef struct {
  float x;
  float y;
  float z;
} fvector_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} ivector16_t;

void Quaternion_init(quaternion_t *q) {
  q->w = 1.0f;
  q->x = 0.0f;
  q->y = 0.0f;
  q->z = 0.0f;
}

void Quaternion_init_values(quaternion_t *q, float nw, float nx, float ny,
                            float nz) {
  q->w = nw;
  q->x = nx;
  q->y = ny;
  q->z = nz;
}

quaternion_t Quaternion_getProduct(quaternion_t *q1, quaternion_t *q2) {
  quaternion_t result;
  result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
  return result;
}

quaternion_t Quaternion_getConjugate(quaternion_t *q) {
  quaternion_t result;
  result.w = q->w;
  result.x = -q->x;
  result.y = -q->y;
  result.z = -q->z;
  return result;
}

float Quaternion_getMagnitude(quaternion_t *q) {
  return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

void Quaternion_normalize(quaternion_t *q) {
  float m = Quaternion_getMagnitude(q);
  q->w /= m;
  q->x /= m;
  q->y /= m;
  q->z /= m;
}

quaternion_t Quaternion_getNormalized(quaternion_t *q) {
  quaternion_t result = *q;
  Quaternion_normalize(&result);
  return result;
}

void VectorInt16_init(ivector16_t *v) {
  v->x = 0;
  v->y = 0;
  v->z = 0;
}

void VectorInt16_init_values(ivector16_t *v, int16_t nx, int16_t ny,
                             int16_t nz) {
  v->x = nx;
  v->y = ny;
  v->z = nz;
}

float VectorInt16_getMagnitude(ivector16_t *v) {
  return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

void VectorInt16_normalize(ivector16_t *v) {
  float m = VectorInt16_getMagnitude(v);
  v->x /= m;
  v->y /= m;
  v->z /= m;
}

ivector16_t VectorInt16_getNormalized(ivector16_t *v) {
  ivector16_t result = *v;
  VectorInt16_normalize(&result);
  return result;
}

void VectorInt16_rotate(ivector16_t *v, quaternion_t *q) {
  quaternion_t p = {0, v->x, v->y, v->z};
  p = Quaternion_getProduct(q, &p);
  quaternion_t c = Quaternion_getConjugate(q);
  p = Quaternion_getProduct(&p, &c);
  v->x = p.x;
  v->y = p.y;
  v->z = p.z;
}

ivector16_t VectorInt16_getRotated(ivector16_t *v, quaternion_t *q) {
  ivector16_t result = *v;
  VectorInt16_rotate(&result, q);
  return result;
}

void VectorFloat_init(fvector_t *v) {
  v->x = 0;
  v->y = 0;
  v->z = 0;
}

void VectorFloat_init_values(fvector_t *v, float nx, float ny, float nz) {
  v->x = nx;
  v->y = ny;
  v->z = nz;
}

float VectorFloat_getMagnitude(fvector_t *v) {
  return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

void VectorFloat_normalize(fvector_t *v) {
  float m = VectorFloat_getMagnitude(v);
  v->x /= m;
  v->y /= m;
  v->z /= m;
}

fvector_t VectorFloat_getNormalized(fvector_t *v) {
  fvector_t result = *v;
  VectorFloat_normalize(&result);
  return result;
}

void VectorFloat_rotate(fvector_t *v, quaternion_t *q) {
  quaternion_t p = {0, v->x, v->y, v->z};
  p = Quaternion_getProduct(q, &p);
  quaternion_t c = Quaternion_getConjugate(q);
  p = Quaternion_getProduct(&p, &c);
  v->x = p.x;
  v->y = p.y;
  v->z = p.z;
}

fvector_t VectorFloat_getRotated(fvector_t *v, quaternion_t *q) {
  fvector_t result = *v;
  VectorFloat_rotate(&result, q);
  return result;
}
