/*
 * FOC_math.h
 *
 *  Created on: Jul 12, 2025
 *      Author: munir
 */

#ifndef FOC_INC_FOC_MATH_H_
#define FOC_INC_FOC_MATH_H_


#include <stdint.h>
#include "math.h"
#include "flash.h"

//#include "bldc_quickLut.h"

//extern float sin_lut[LUT_SIZE];
//extern float cos_lut[LUT_SIZE];

/* ------------------------- Trig LUT helpers ------------------------- */
void  Init_Trig_Lut(void);
void  Norm_Angle_Rad(float *theta);
float Fast_Sin(float theta);
float Fast_Cos(float theta);
void  Pre_Calc_Sin_Cos(float theta, float *sin_theta, float *cos_theta);

/* ------------------------- FOC transforms --------------------------- */
void Clarke_Transform(float ia, float ib, float *i_alpha, float *i_beta);
void Park_Transform(float i_alpha, float i_beta, float sin_theta, float cos_theta,
                    float *id, float *iq);
void Clarke_Park_Transform(float ia, float ib, float sin_theta, float cos_theta,
                           float *id, float *iq);
void Inverse_Park_Transform(float vd, float vq, float sin_theta, float cos_theta,
                            float *valpha, float *vbeta);
void Inverse_Clarke_Transform(float valpha, float vbeta, float *va, float *vb, float *vc);
void SVPWM(float valpha, float vbeta, float vbus, uint32_t pwm_period,
           uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w);

/* ------------------------- Measurement ------------------------------ */
void Vabc_To_Vdq(float va, float vb, float vc, float sin_theta, float cos_theta,
                 float *vd, float *vq);

#endif /* FOC_INC_FOC_MATH_H_ */
