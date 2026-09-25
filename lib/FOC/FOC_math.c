/*
 * FOC_math.c
 *
 *  Created on: Jul 12, 2025
 *      Author: munir
 */

#include "FOC_math.h"

float Sin_Lut[LUT_SIZE];
float Cos_Lut[LUT_SIZE];

/* ------------------------- Trig LUT helpers ------------------------- */
void Init_Trig_Lut(void) {
    for (int i = 0; i < LUT_SIZE; ++i) {
        float angle = i * LUT_STEP;
        Sin_Lut[i] = sinf(angle);
        Cos_Lut[i] = cosf(angle);
    }
}

void Norm_Angle_Rad(float *theta) {
    while (*theta < 0) *theta += TWO_PI;
    while (*theta >= TWO_PI) *theta -= TWO_PI;
}

float Fast_Sin(float theta) {
    Norm_Angle_Rad(&theta);
    float index_f = theta / LUT_STEP;
    int index = (int)index_f;
    float frac = index_f - index;

    int next_index = (index + 1) % LUT_SIZE;

    return Sin_Lut[index] * (1.0f - frac) + Sin_Lut[next_index] * frac;
}

float Fast_Cos(float theta) {
    Norm_Angle_Rad(&theta);
    float index_f = theta / LUT_STEP;
    int index = (int)index_f;
    float frac = index_f - index;

    int next_index = (index + 1) % LUT_SIZE;

    return Cos_Lut[index] * (1.0f - frac) + Cos_Lut[next_index] * frac;
}

void Pre_Calc_Sin_Cos(float theta, float *sin_theta, float *cos_theta) {
    *sin_theta = Fast_Sin(theta);
    *cos_theta = Fast_Cos(theta);
}

/* ------------------------- FOC transforms --------------------------- */
void Clarke_Transform(float ia, float ib, float *i_alpha, float *i_beta) {
    // Clarke transform
    *i_alpha = ia;
    *i_beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
}

void Park_Transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float *id, float *iq) {
    // Park transform
    *id = i_alpha * cos_theta + i_beta * sin_theta;
    *iq = i_beta * cos_theta - i_alpha * sin_theta;
}

// Fast combined Clarke + Park Transform
void Clarke_Park_Transform(float ia, float ib, float sin_theta, float cos_theta, float *id, float *iq) {
    // Clarke transform
    float i_alpha = ia;
    float i_beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;

    // Park transform
    *id = i_alpha * cos_theta + i_beta * sin_theta;
    *iq = i_beta * cos_theta - i_alpha * sin_theta;
}

// Inverse Park Transform
void Inverse_Park_Transform(float vd, float vq, float sin_theta, float cos_theta, float *valpha, float *vbeta) {
    *valpha = vd * cos_theta - vq * sin_theta;
    *vbeta  = vd * sin_theta + vq * cos_theta;
}

// Inverse Clarke Transform
void Inverse_Clarke_Transform(float valpha, float vbeta, float *va, float *vb, float *vc) {
    *va = valpha;
    *vb = -0.5f * valpha + SQRT3_BY_TWO * vbeta;   // cos(120°), sin(120°)
    *vc = -0.5f * valpha - SQRT3_BY_TWO * vbeta;
}

void Vabc_To_Vdq(float va, float vb, float vc, float sin_theta, float cos_theta, float *vd, float *vq) {
    float v0 = (va + vb + vc) * 0.3333333f;  // 1/3
    float v_alpha = va - v0;
    float v_beta = ONE_BY_SQRT3 * (vb - vc);

    *vd = v_alpha * cos_theta + v_beta * sin_theta;
    *vq = v_beta * cos_theta - v_alpha * sin_theta;
}

/**
 * @brief Space Vector PWM Modulation
 * @param valpha Alpha component of voltage vector
 * @param vbeta Beta component of voltage vector
 * @param vbus DC bus voltage
 * @param pwm_period Full PWM period value
 * @param pwm_u Output duty cycle for phase U (0 to pwm_period)
 * @param pwm_v Output duty cycle for phase V
 * @param pwm_w Output duty cycle for phase W
 */
void SVPWM(float valpha, float vbeta, float vbus, uint32_t pwm_period,
          uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w)
{
    // 1. Normalize voltages by vbus
    float alpha = valpha / vbus;
    float beta = vbeta / vbus;

    // 2. Sector determination
    uint8_t sector;
    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            sector = (ONE_BY_SQRT3 * beta > alpha) ? 2 : 1;  // 1/sqrt(3) ≈ 0.577
        } else {
            sector = (-ONE_BY_SQRT3 * beta > alpha) ? 3 : 2;
        }
    } else {
        if (alpha >= 0.0f) {
            sector = (-ONE_BY_SQRT3 * beta > alpha) ? 5 : 6;
        } else {
            sector = (ONE_BY_SQRT3 * beta > alpha) ? 4 : 5;
        }
    }

    // 3. Calculate active vector times
    int32_t t1, t2;
    switch(sector) {
        case 1:
            t1 = (int32_t)((alpha - ONE_BY_SQRT3 * beta) * pwm_period);
            t2 = (int32_t)(TWO_BY_SQRT3 * beta * pwm_period);
            *pwm_u = (pwm_period + t1 + t2) / 2;
            *pwm_v = *pwm_u - t1;
            *pwm_w = *pwm_v - t2;
            break;

        case 2:
            t1 = (int32_t)((alpha + ONE_BY_SQRT3 * beta) * pwm_period);
            t2 = (int32_t)((-alpha + ONE_BY_SQRT3 * beta) * pwm_period);
            *pwm_v = (pwm_period + t1 + t2) / 2;
            *pwm_u = *pwm_v - t2;
            *pwm_w = *pwm_u - t1;
            break;

        case 3:
            t1 = (int32_t)(TWO_BY_SQRT3 * beta * pwm_period);
            t2 = (int32_t)((-alpha - ONE_BY_SQRT3 * beta) * pwm_period);
            *pwm_v = (pwm_period + t1 + t2) / 2;
            *pwm_w = *pwm_v - t1;
            *pwm_u = *pwm_w - t2;
            break;

        case 4:
            t1 = (int32_t)((-alpha + ONE_BY_SQRT3 * beta) * pwm_period);
            t2 = (int32_t)(-TWO_BY_SQRT3 * beta * pwm_period);
            *pwm_w = (pwm_period + t1 + t2) / 2;
            *pwm_v = *pwm_w - t2;
            *pwm_u = *pwm_v - t1;
            break;

        case 5:
            t1 = (int32_t)((-alpha - ONE_BY_SQRT3 * beta) * pwm_period);
            t2 = (int32_t)((alpha - ONE_BY_SQRT3 * beta) * pwm_period);
            *pwm_w = (pwm_period + t1 + t2) / 2;
            *pwm_u = *pwm_w - t1;
            *pwm_v = *pwm_u - t2;
            break;

        case 6:
            t1 = (int32_t)(-TWO_BY_SQRT3 * beta * pwm_period);
            t2 = (int32_t)((alpha + ONE_BY_SQRT3 * beta) * pwm_period);
            *pwm_u = (pwm_period + t1 + t2) / 2;
            *pwm_w = *pwm_u - t2;
            *pwm_v = *pwm_w - t1;
            break;
    }

    // 4. Clamp outputs to valid range
    *pwm_u = (*pwm_u > pwm_period) ? pwm_period : *pwm_u;
    *pwm_v = (*pwm_v > pwm_period) ? pwm_period : *pwm_v;
    *pwm_w = (*pwm_w > pwm_period) ? pwm_period : *pwm_w;
}
