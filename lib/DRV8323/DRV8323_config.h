/*
 * DRV8323_config.h
 *
 *  Created on: Apr 2026
 *      Author: Ngoc
 *
 *  Register bit fields verified against DRV832x datasheet SLVSDJ3D (Rev D, March 2022)
 */

#ifndef DRV8323_DRIVER_INC_DRV8323_CONFIG_H_
#define DRV8323_DRIVER_INC_DRV8323_CONFIG_H_

/* --------- DRV8323S SPI frame format (16-bit) ---------
 *  Bit 15   : R/W  (1 = read, 0 = write)
 *  Bits 14:11: Register address [3:0]
 *  Bits 10:0 : Data [10:0]
 */
#define DRV8323_SPI_READ            (1U << 15)
#define DRV8323_SPI_WRITE           (0U << 15)
#define DRV8323_SPI_ADDR_SHIFT      11U
#define DRV8323_SPI_DATA_MASK       0x07FFU // 0000 0111 1111 1111

/* --------- Register addresses --------- */
#define DRV8323_REG_FAULT_STAT1     0x00U
#define DRV8323_REG_FAULT_STAT2     0x01U
#define DRV8323_REG_DRV_CTRL        0x02U
#define DRV8323_REG_GATE_HS         0x03U
#define DRV8323_REG_GATE_LS         0x04U
#define DRV8323_REG_OCP_CTRL        0x05U
#define DRV8323_REG_CSA_CTRL        0x06U

/* =========================================================
 * Driver Control Register (0x02)
 * ---------------------------------------------------------
 * Bit 10   : Reserved
 * Bit  9   : DIS_CPUV(Disable Charge Pump Undervoltage)	0=charge pump UVLO fault enabled,	1=disabled
 * Bit  8   : DIS_GDF(Disable Gate Drive Fault)   			0=gate drive fault enabled,      	1=disabled
 * Bit  7   : OTW_REP(Over-Temperature Warning Report)   	0=OTW dose not report nFAULT,  		1=OTW reports nFAULT
 * Bit 6:5  : PWM_MODE  00=6x PWM, 01=3x PWM, 10=1x PWM, 11=Independent
 * Bit  4   : 1PWM_COM  0=synchronous rectification, 1=asynchronous (diode freewheel)
 * Bit  3   : 1PWM_DIR  direction bit trong 1x PWM mode
 * Bit  2   : COAST     1=tất cả MOSFET Hi-Z
 * Bit  1   : BRAKE     1=bật tất cả low-side FET (chỉ dùng trong 1x PWM mode)
 * Bit  0   : CLR_FLT   write 1 để clear latched fault, tự reset sau khi write
 * ========================================================= */
#define DRV_CTRL_DIS_CPUV(x)	((0x0U | x) << 9)	// default bit 0
#define DRV_CTRL_DIS_GDF(x)		((0x0U | x) << 8)   // default bit 0
#define DRV_CTRL_OTW_REP(x)		((0x0U | x) << 7)   // default bit 0
#define DRV_CTRL_PWM_MODE(x)    ((0x0U | x) << 5)   // default 00: 0=6PWM, 1=3PWM, 2=1PWM, 3=Independent
#define DRV_CTRL_1PWM_COM(x)	((0x0U | x) << 4)   // default bit 0
#define DRV_CTRL_1PWM_DIR(x)	((0x0U | x) << 3)   // default bit 0
#define DRV_CTRL_COAST(x)		((0x0U | x) << 2)   // default bit 0
#define DRV_CTRL_BRAKE(x)		((0x0U | x) << 1)   // default bit 0
#define DRV_CTRL_CLR_FLT(x)		((0x0U | x) << 0)   // write 1 to clear latched fault

#define ADD_0x02_PRESET \
    (DRV_CTRL_DIS_CPUV	(0U) | /* Charge pump UVLO fault is enabled */ \
     DRV_CTRL_DIS_GDF	(0U) | /* Gate drive fault is enabled */ \
     DRV_CTRL_OTW_REP	(0U) | /* OTW is not reported on nFAULT or the FAULT bit */ \
     DRV_CTRL_PWM_MODE	(0U) | /* 6x PWM Mode */ \
     DRV_CTRL_1PWM_COM	(0U) | /* 0b = 1x PWM mode uses synchronous rectification */ \
     DRV_CTRL_1PWM_DIR	(0U) | \
     DRV_CTRL_COAST		(0U) | \
     DRV_CTRL_BRAKE		(0U) | \
     DRV_CTRL_CLR_FLT	(1U))

/* =========================================================
 * Gate Drive HS Register (0x03)
 * ---------------------------------------------------------
 * Bit 10:8 : LOCK      3=unlock, 6=lock
 * Bit  7:4 : IDRIVEP   source current (charge gate)
 * Bit  3:0 : IDRIVEN   sink current   (discharge gate)
 *
 * Use for both GATE_HS and GATE_LS
 * IDRIVEP: 0=10mA,  1=30mA,   2=60mA,   3=80mA,   4=120mA,  5=140mA,
 *          6=170mA, 7=190mA,  8=260mA,  9=330mA,  10=370mA, 11=440mA,
 *          12=570mA,13=680mA, 14=820mA, 15=1000mA
 * IDRIVEN: 0=20mA,  1=60mA,   2=120mA,  3=160mA,  4=240mA,  5=280mA,
 *          6=340mA, 7=380mA,  8=520mA,  9=660mA,  10=740mA, 11=880mA,
 *          12=1140mA,13=1360mA,14=1640mA,15=2000mA
 *
 * AON6354: Qg(10V)=35nC max, target t_rise=150ns @ 10kHz FOC:
 *   I_source = 35nC / 150ns = 233mA  → index 8 = 260mA
 *   I_sink   = 2× source            → index 8 = 520mA
 * ========================================================= */
#define GATE_HS_LOCK(x)		((0x0U | x) << 8)	// default 011: 3
#define GATE_HS_IDRIVEP(x)	((0x0U | x) << 4)	// default 1111: 15
#define GATE_HS_IDRIVEN(x)	((0x0U | x) << 0)	// default 1111: 15

#define ADD_0x03_PRESET \
    (GATE_HS_LOCK		(3U) | /* Unlock */ \
     GATE_HS_IDRIVEP	(4U) | /* 0100b = 120mA */ \
     GATE_HS_IDRIVEN	(4U))  /* 0100b = 240mA */

/* =========================================================
 * Gate Drive LS Register (0x04)
 * ---------------------------------------------------------
 * Bit 10   : CBC        1=cycle-by-cycle OCP, 0=latched OCP
 * Bit  9:8 : TDRIVE     0=500ns, 1=1us, 2=2us, 3=4us
 * Bit  7:4 : IDRIVEP    source current (enum giống HS)
 * Bit  3:0 : IDRIVEN    sink current   (enum giống HS)
 * ========================================================= */
#define GATE_LS_CBC(x)		((0x0U | x) << 10)	// default bit 1
#define GATE_LS_TDRIVE(x)	((0x0U | x) << 8)	// default 11: 3
#define GATE_LS_IDRIVEP(x)	((0x0U | x) << 4)	// default 1111: 15
#define GATE_LS_IDRIVEN(x)	((0x0U | x) << 0)	// default 1111: 15


#define ADD_0x04_PRESET \
    (GATE_LS_CBC		(1U) | \
     GATE_LS_TDRIVE		(0U) | /* 00b = 500ns peak gate-current drive time */ \
     GATE_LS_IDRIVEP	(4U) | /* 0100b = 120mA */ \
     GATE_LS_IDRIVEN	(4U))  /* 0100b = 240mA */

/* =========================================================
 * OCP Control Register (0x05)
 * ---------------------------------------------------------
 * Bit 10   : TRETRY     0=4ms retry, 1=50ms retry
 * Bit  9:8 : DEAD_TIME  0=50ns, 1=100ns, 2=200ns, 3=400ns
 * Bit  7:6 : OCP_MODE   0=latched, 1=auto-retry, 2=report only, 3=disabled
 * Bit  5:4 : OCP_DEG    0=2us, 1=4us, 2=6us, 3=8us  (deglitch time)
 * Bit  3:0 : VDS_LVL    0=0.06V, 1=0.13V, 2=0.20V,  3=0.26V,
 *                       4=0.31V, 5=0.45V, 6=0.53V,  7=0.60V,
 *                       8=0.68V, 9=0.75V, 10=0.94V, 11=1.13V,
 *                       12=1.30V,13=1.50V, 14=1.69V, 15=1.88V
 * ========================================================= */
#define OCP_TRETRY(x)		((0x0U | x) << 10)	// default bit 0
#define OCP_DEAD_TIME(x)	((0x0U | x) << 8)	// default 01: 1
#define OCP_MODE(x)			((0x0U | x) << 6)	// default 01: 1
#define OCP_DEG(x)			((0x0U | x) << 4)	// default 01: 1
#define OCP_VDS_LVL(x)		((0x0U | x) << 0)	// default 1001: 9

#define ADD_0x05_PRESET \
    (OCP_TRETRY		(1U) | /* 1b = VDS_OCP and SEN_OCP retry time is 50µs */ \
     OCP_DEAD_TIME	(1U) | /* 10b = 200ns dead time */ \
     OCP_MODE		(2U) | /* 01b = Overcurrent causes an automatic retrying fault */ \
     OCP_DEG		(1U) | /* 01b = Overcurrent deglitch time of 4µs */ \
     OCP_VDS_LVL	(9U))  /* 0000b = 0.06V */

/* =========================================================
 * CSA Control Register (0x06)
 * =========================================================
 * Bit(s)   Name        Description
 * --------------------------------------------------------------------------
 * 10       CSA_FET     Current Sense Amplifier Input Selection
 *                      0b = Positive input is SPx (low-side shunt)
 *                      1b = Positive input is SHx (VDS sensing, also sets LS_REF=1)
 * 9        VREF_DIV    CSA Reference Voltage Select
 *                      0b = VREF (unidirectional mode)
 *                      1b = VREF/2 (bidirectional mode)  [DEFAULT]
 * 8        LS_REF      Low-Side MOSFET VDS_OCP Measurement Reference
 *                      0b = Measured across SHx to SPx  [DEFAULT]
 *                      1b = Measured across SHx to SNx
 * 7:6      CSA_GAIN    Current Sense Amplifier Gain
 *                      1 = 5 V/V, 2 = 10 V/V, 3 = 20 V/V [DEFAULT], 3 = 40 V/V
 * 5        DIS_SEN     Sense Overcurrent Fault
 *                      0b = OCP enabled  [DEFAULT]
 *                      1b = OCP disabled
 * 4        CSA_CAL_A   Phase A Current Sense Amplifier Calibration
 *                      0b = Normal operation  [DEFAULT]
 *                      1b = Short inputs for offset calibration
 * 3        CSA_CAL_B   Phase B Current Sense Amplifier Calibration
 *                      0b = Normal operation  [DEFAULT]
 *                      1b = Short inputs for offset calibration
 * 2        CSA_CAL_C   Phase C Current Sense Amplifier Calibration
 *                      0b = Normal operation  [DEFAULT]
 *                      1b = Short inputs for offset calibration
 * 1:0      SEN_LVL     Sense OCP Voltage Level
 *                      0 = 0.25 V, 1 = 0.5 V, 2 = 0.75 V, 3 = 1.0 V  [DEFAULT]
 * =========================================================== */

#define CSA_FET(x)		((0x0U | x) << 10)	// default bit 0
#define CSA_VREF_DIV(x)	((0x0U | x) << 9)	// default bit 1
#define CSA_LS_REF(x)	((0x0U | x) << 8)	// default bit 0
#define CSA_GAIN(x)		((0x0U | x) << 6)	// default 2: 10
#define CSA_DIS_SEN(x)	((0x0U | x) << 5)	// default bit 0
#define CSA_CAL_A(x)	((0x0U | x) << 4)	// default bit 0
#define CSA_CAL_B(x)	((0x0U | x) << 3)	// default bit 0
#define CSA_CAL_C(x)	((0x0U | x) << 2)	// default bit 0
#define CSA_SEN_LVL(x)	((0x0U | x) << 0)	// default 3: 11

#define ADD_0x06_PRESET \
    (CSA_FET		(0U) | /* 0b = Current sense amplifier positive input is SPx */ \
     CSA_VREF_DIV	(1U) | /* 1b = Current sense amplifier reference voltage is VREF divided by 2 */ \
     CSA_LS_REF		(0U) | /* 0b = VDS_OCP for the low-side MOSFET is measured across SHx to SPx */ \
     CSA_GAIN		(1U) | /* 01b = 10V/V current sense amplifier gain */ \
     CSA_DIS_SEN	(1U) | /* 1b = Sense overcurrent fault is disabled */ \
     CSA_CAL_A		(0U) | /* 0b = Normal current sense amplifier A operation */ \
     CSA_CAL_B		(0U) | /* 0b = Normal current sense amplifier B operation */ \
     CSA_CAL_C		(0U) | /* 0b = Normal current sense amplifier C operation */ \
     CSA_SEN_LVL	(3U))  /* 11b = Sense OCP 1V */

/* =========================================================
 * Default register values at init
 * Target: AON6354 MOSFET, FOC 10kHz, AVDD=3.3V
 * ========================================================= */

#endif /* DRV8323_DRIVER_INC_DRV8323_CONFIG_H_ */
