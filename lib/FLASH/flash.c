#include "flash.h"
#include <string.h>

void flash_default_config(motor_config_t *data) {
    data->id_kp = IQ_KP;
    data->id_ki = IQ_KI;
    data->id_out_max = 0.88f;
    data->id_e_deadband = 0.0001f;

    data->iq_kp = IQ_KP;
    data->iq_ki = IQ_KI;
    data->iq_out_max = 0.88f;
    data->iq_e_deadband = 0.0001f;

    data->I_ctrl_bandwidth = 50.0f;

    data->speed_kp = 0.01f;
    data->speed_ki = 0.1f;
    data->speed_kd = 0.0001f;
    data->speed_out_max = 10.0f;
    data->speed_e_deadband = 0.01f;

    data->pos_kp = POS_KP;
    data->pos_ki = 0.0;
    data->pos_kd = POS_KD;
    data->pos_out_max = 10.0f;
    data->pos_e_deadband = 0.00174f; // 0.1deg

    data->encd_offset = 0.0f;
    memset(data->encd_error_comp, 0, sizeof(data->encd_error_comp));

    data->freq = 10000;
//    data->dir = NORMAL_DIR;
    data->gear_ratio = 1.0f;

    data->Rs = 0.26f;
    data->Ld = 0.000160f;
    data->Lq = 0.000160f;
}

//void update_encd_error_comp(motor_config_t *data) {
//    memcpy(data->encd_error_comp, encd_error_comp, sizeof(data->encd_error_comp));
//}

void flash_auto_tuning_torque_control(motor_config_t *data) {
  if (data->Rs <= 0.0f || data->Rs > 3.0f ||
      data->Ld <= 0.0f || data->Ld > 3.0f ||
      data->Lq <= 0.0f || data->Lq > 3.0f ||
      data->I_ctrl_bandwidth <= 0.0f)
      return;

  float omega = TWO_PI * data->I_ctrl_bandwidth;

  data->id_kp = data->Ld * omega;
  data->id_ki = data->Rs * omega;
  data->iq_kp = data->Lq * omega;
  data->iq_ki = data->Rs * omega;
}
