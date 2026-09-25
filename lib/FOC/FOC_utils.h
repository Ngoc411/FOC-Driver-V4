/*
 * FOC_utils.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef FOC_INC_FOC_UTILS_H_
#define FOC_INC_FOC_UTILS_H_

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "FOC_math.h"
#include "pid_utils.h"
#include "flash.h"

/* extern variable - for RL measured */
extern _Bool foc_ready;
extern float Vd_buff[MAX_I_SAMPLE];
extern float Vq_buff[MAX_I_SAMPLE];
extern float Id_buff[MAX_I_SAMPLE];
extern float Iq_buff[MAX_I_SAMPLE];

typedef enum {
	POWER_UP_MODE,
	TORQUE_CONTROL_MODE,
	SPEED_CONTROL_MODE,
	POSITION_CONTROL_MODE,
	CALIBRATION_MODE,
	MEASURE_RL_MODE,
	TEST_MODE,
}motor_mode_t;

typedef enum {
  RS, LD, LQ
}inject_taregt_t;

typedef struct {
    /* ---------- Motor parameters (nameplate) ---------- */
    float Rs;
    float Ld;
    float Lq;

    /* ---------- R, L measurement (injection) ---------- */
    float meas_inj_freq;
    float meas_inj_amp;
    float meas_inj_omega;
    float meas_inj_dc_hold;      // áp d-DC giữ rotor tại theta_e=0 khi đo L (chống lẫn d/q)
    inject_taregt_t meas_inj_target;
    int meas_inj_n;
    _Bool meas_inj_start_flag;

    /* ---------- PWM ---------- */
    volatile uint32_t *pwm_a;
    volatile uint32_t *pwm_b;
    volatile uint32_t *pwm_c;
    uint32_t pwm_res;

    /* ---------- Electrical signals (FOC) ---------- */
    // abc frame
    float va, vb, vc;
    float ia, ib, ic;
    // alpha-beta frame
    float v_alpha, v_beta;
    float i_alpha, i_beta;
    // dq frame
    float id, iq;
    float id_filtered, iq_filtered;
    // bus
    float v_bus;
    float i_bus;

    /* ---------- Position: single turn ---------- */
    float m_rad_raw;             // mechanical angle without offset
    float m_rad;                 // mechanical angle with offset
    float m_rad_offset;
    float e_rad;                 // electrical angle
    float e_rad_comp;            // electrical angle with compensation

    /* ---------- Position: multi-turn / sensors ---------- */
    float actual_angle_mt6816;
    float actual_angle_as5600;
    float *actual_angle_general;
    float actual_angle_mt6816_offset;
    uint8_t check_mul_round;
    float mutil_turn_deg;
    float *mech_theta;

    /* ---------- Velocity ---------- */
    float encd_speed;
    float actual_speed;

    /* ---------- References / setpoints ---------- */
    float id_ref, iq_ref;
    float tau_ref;

    /* ---------- Controllers / tuning ---------- */
    float I_ctrl_bandwidth;
    PID_Controller_t id_ctrl, iq_ctrl;
    PID_Controller_t speed_ctrl;
    PID_Controller_t pos_ctrl;
    motor_mode_t control_mode;

    /* ---------- Misc ---------- */
    uint8_t loop_count;
} FOC_t;

/* ------------------------- PWM init ----------------------------- */
void FOC_Pwm_Init(FOC_t *foc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c, uint32_t pwm_res);
/* ------------------------- Sensor init ----------------------------- */
void FOC_Sensor_Init(FOC_t *foc, float m_rad_offset);
/* ------------------------- FOC control ----------------------------- */
void FOC_Current_Control_Update(FOC_t *foc);
void FOC_Speed_Control_Update(FOC_t *foc, float speed_ref);
void FOC_Position_Control_Update(FOC_t *foc, float pos_ref);
/* ------------------------- Encoder cal ----------------------------- */
void FOC_Calc_Mech_Speed_Encoder(FOC_t *foc);
void FOC_Calc_Mech_Pos_Encoder(FOC_t *foc);
void FOC_Calc_Electric_Angle(FOC_t *foc);
void FOC_Cal_Encoder_Misalignment(FOC_t *foc);
void FOC_Cal_Encoder(FOC_t *foc);
void FOC_Open_Loop_Voltage_Control(FOC_t *foc, float vd_ref, float vq_ref, float angle_rad);
void FOC_Meas_Inj_Dq_Process(FOC_t *foc, float ts);
void FOC_Estimate_Resistance(FOC_t *foc);
void FOC_Estimate_Inductance(FOC_t *foc, float ts);

#endif /* FOC_INC_FOC_UTILS_H_ */
