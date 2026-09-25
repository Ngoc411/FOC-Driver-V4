/*
 * DRV8323.h
 *
 *  DRV8323SRTAT gate driver — SPI-configurable 3-phase MOSFET driver
 *
 *  SPI3 interface: PC10(SCK), PC11(MISO), PC12(MOSI), PD2(nSCS - software cs)
 *
 *  Created on: Apr 2026
 *      Author: Ngoc
 */

#ifndef DRV8323_DRIVER_INC_DRV8323_H_
#define DRV8323_DRIVER_INC_DRV8323_H_

#include "DRV8323_config.h"
#include "stm32g4xx_hal.h"
#include <stddef.h>
#include <math.h>
#include "flash.h"

/* ------------------------------------------------------------------ */
/*  Driver handle                                                     */
/* ------------------------------------------------------------------ */
typedef struct {
    /* TIMER config */

    /* SPI interface for register access */
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;

    /* Gate-enable GPIO */
    GPIO_TypeDef      *engate_port;
    uint16_t           engate_pin;

    /* nFAULT GPIO (active-low input, open-drain from DRV8323) */
    GPIO_TypeDef      *nfault_port;
    uint16_t           nfault_pin;

    /* PWM timer */
    TIM_HandleTypeDef *timer;
    uint32_t           pwm_freq;
    uint32_t           pwm_resolution;   /* = ARR value */

    /* ADC raw values set by ISR */
    uint32_t adc_a;
    uint32_t adc_c;

    /* Current sensing */
    int gain;
    float v_to_current;  /* = 1.0f / (gain * R_shunt) */

    uint8_t check_spi_cfg;
} DRV8323_t;

typedef struct {
    uint16_t fault1;
    uint16_t fault2;
    uint16_t drv_ctrl;
    uint16_t gate_hs;
    uint16_t gate_ls;
    uint16_t ocp_ctrl;
    uint16_t csa_ctrl;
    uint8_t  nfault_pin;
    uint8_t  engate_pin;
} DRV8323_dbg_t;

/* ------------------------------------------------------------------ */
/*  Inline helpers                                                    */
/* ------------------------------------------------------------------ */
static inline void DRV8323_Enable_Gate(DRV8323_t *drv)
{
    drv->engate_port->BSRR = drv->engate_pin; // HIGH
}

static inline void DRV8323_Disable_Gate(DRV8323_t *drv)
{
    drv->engate_port->BSRR = (uint32_t)(drv->engate_pin) << 16; // LOW
}

static inline void DRV8323_Set_Adc_A(DRV8323_t *drv, uint32_t val) { drv->adc_a = val; }
static inline void DRV8323_Set_Adc_C(DRV8323_t *drv, uint32_t val) { drv->adc_c = val; }

static inline uint8_t DRV8323_Fault_Active(DRV8323_t *drv)
{
    /* nFAULT is active-low — fault when pin reads 0 */
    return (drv->nfault_port->IDR & drv->nfault_pin) == 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

/* ------------------------- Configuration ------------------------- */
void DRV8323_SPI_Config(DRV8323_t *drv,
                        SPI_HandleTypeDef *spi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin);

void DRV8323_GPIO_ENGATE_Config(DRV8323_t *drv,
                                GPIO_TypeDef *port,
                                uint16_t pin);

void DRV8323_GPIO_NFAULT_Config(DRV8323_t *drv,
                                GPIO_TypeDef *port,
                                uint16_t pin);

int  DRV8323_TIMER_Config(DRV8323_t *drv,
                          TIM_HandleTypeDef *timer,
                          uint32_t freq);

int  DRV8323_Current_Sens_Config(DRV8323_t *drv);

/* ------------------------- Drv setup ----------------------------- */
int  DRV8323_Init(DRV8323_t *drv, DRV8323_dbg_t *dbg);

/* ------------------------- PWM control --------------------------- */
int  DRV8323_Start_Pwm(DRV8323_t *drv);
int  DRV8323_Stop_Pwm(DRV8323_t *drv);
void DRV8323_Set_Pwm(DRV8323_t *drv, uint32_t pwma, uint32_t pwmb, uint32_t pwmc);

/* ------------------------- Get current --------------------------- */
void DRV8323_Get_Current(DRV8323_t *drv, float *ia, float *ic);

/* ------------------------- SPI r/w ------------------------------- */
uint16_t DRV8323_Reg_Read(DRV8323_t *drv, uint8_t addr);
void     DRV8323_Reg_Write(DRV8323_t *drv, uint8_t addr, uint16_t data);

/* ------------------------- SPI faults ---------------------------- */
void DRV8323_Read_Fault1(DRV8323_t *drv, DRV8323_dbg_t *dbg);
void DRV8323_Read_Fault2(DRV8323_t *drv, DRV8323_dbg_t *dbg);
void DRV8323_Clear_Faults(DRV8323_t *drv);
void DRV8323_Debug_Read(DRV8323_t *drv, volatile DRV8323_dbg_t *dbg);

#endif /* DRV8323_DRIVER_INC_DRV8323_H_ */
