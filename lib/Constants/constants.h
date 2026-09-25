#ifndef CONSTANTS_H
#define CONSTANTS_H

// CAN
#define NODE_ID					0
#define CAN_FEEDBACK_POS_VEL	50
#define CAN_FEEDBACK_PA_PB		51
#define CAN_FEEDBACK_PC_TAU		52
#define CAN_FEEDBACK_TEMP_VBUS	53
#define CAN_FEEDBACK_STATE		54
#define CAN_ACTIONS				(60 + NODE_ID)

// DRV8323
#define PWM_FRE					20000
#define ADC_RES     			4096
#define VCC         			3.32f
#define ADC_2_VOLT				(VCC / ADC_RES)
#define ADC_2_POWER_VOLT		0.01651611328125f
#define CURRENT_FILTER_ALPHA	0.466512f
#define R_SHUNT					0.01f
#define V_OFFSET				(VCC / 2)

// Control mode
/* RAD_CTRL: 1 = toàn hệ điều khiển chạy đơn vị SI (rad, rad/s) — lệnh, giới hạn,
 *               feedback, homing/bảo vệ và các hàm *_rad đều dùng rad end-to-end,
 *               KHÔNG quy đổi trong vòng lặp.
 *           0 = giữ hành vi cũ (deg, rpm) — dùng các hàm gốc.
 * Định nghĩa tại đây để mọi file (main + lib) cùng thấy. */
#define RAD_CTRL 1
#if RAD_CTRL
#define UANG(deg)				DEG_TO_RAD_CONST(deg)
#define UVEL(rpm)				RPM_TO_RAD_S_CONST(rpm)
#define MAX_POS					2*PI
#define MIN_POS					-2*PI
#define MAX_SPEED				RPM_TO_RAD_S_CONST(2800.0f)/GEAR_RATIO
#define MIN_SPEED				-RPM_TO_RAD_S_CONST(2800.0f)/GEAR_RATIO
#else
#define UANG(deg)				(deg)
#define UVEL(rpm)				(rpm)
#define MAX_POS					360.0f
#define MIN_POS					-360.0f
#define MAX_SPEED				2800.0f/GEAR_RATIO
#define MIN_SPEED				-2800.0f/GEAR_RATIO
#endif
#define SPEED_CONTROL_CYCLE		10
#define POS_CONTROL_CYCLE		10

// Actuator specs
//#define POLE_PAIRS              7
//#define GEAR_RATIO 				30.0f // with each node
//#define KT						0.02603f // with each node
//#define DIR						NORMAL_DIR // with each node
//#define MAX_CURRENT				10.0f

// FOC
#define FOC_TS                  (1.0f / (float)PWM_FRE)
#define LUT_SIZE				(1024*4)
#define LUT_STEP				(TWO_PI / (float)LUT_SIZE)

#define TWO_BY_SQRT3			1.15470053838f
#define ONE_BY_SQRT3			0.57735026919f
#define SQRT3_BY_TWO			0.86602540378f
#define SQRT3					1.73205080757f
#define TWO_PI					6.2831853f
#define PI						3.1415926f

static inline float CONSTRAIN(float val, float min, float max) { return (val <= min) ? min : (val >= max ? max : val); }

static inline float RAD_TO_DEG(float rad) { return rad * 57.29577951308232f; } // 180 / π

static inline float DEG_TO_RAD(float deg) { return deg * 0.017453292519943f; } // π / 180

static inline float RPM_TO_RAD_S(float rpm) { return rpm * 0.104719755119660f; } // 2π / 60

static inline float DEG_TO_RAD_CONST(float deg) { return deg * 0.017453292519943f; } // π / 180

static inline float RPM_TO_RAD_S_CONST(float rpm) { return rpm * 0.104719755119660f; } // 2π / 60

static inline float RAD_S_TO_RPM(float rad_s) { return rad_s * 9.549296585513720f; } // 60 / 2π

#define ERROR_LUT_SIZE			(1024)

#define MAG_CAL_RES				(1024*2)
#define MAG_CAL_STEP 			((TWO_PI * POLE_PAIR) / (float)MAG_CAL_RES)

#define CAL_ITERATION 			100

#define VD_CAL 					1.0f
#define VQ_CAL 					0.0f

#define MAX_I_SAMPLE 			128

//  PID cfg - with each node
//#define IQ_KP 					0.214f
//#define IQ_KI 					204.831f
//#define POS_KP 					35.0f
//#define POS_KD 					7.5f

#endif
