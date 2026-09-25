/*
 * DRV8323.c
 *
 *  DRV8323SRTAT gate driver implementation
 *  SPI3: PC10(SCK), PC11(MISO), PC12(MOSI), PD2(nSCS)
 *
 *  Created on: Apr 2026
 *      Author: Ngoc
 */

#include "DRV8323.h"

/* Last HAL status for every SPI transaction — watch in debugger */
volatile HAL_StatusTypeDef drv_spi_wr_status = HAL_ERROR;
volatile HAL_StatusTypeDef drv_spi_rd_status = HAL_ERROR;

/* ------------------------- Internal SPI helpers ------------------------- */
static inline void drv_cs_low(DRV8323_t *drv)
{
    drv->cs_port->BSRR = (uint32_t)(drv->cs_pin) << 16; /* RESET */
}

static inline void drv_cs_high(DRV8323_t *drv)
{
    drv->cs_port->BSRR = drv->cs_pin; /* SET */
}

/* ------------------------- Configuration ------------------------- */

void DRV8323_SPI_Config(DRV8323_t *drv,
                        SPI_HandleTypeDef *spi,
                        GPIO_TypeDef *cs_port,
                        uint16_t cs_pin)
{
    drv->spi = spi;
    drv->cs_port = cs_port;
    drv->cs_pin = cs_pin;
    drv_cs_high(drv);
}

void DRV8323_GPIO_ENGATE_Config(DRV8323_t *drv, GPIO_TypeDef *port, uint16_t pin)
{
    drv->engate_port = port;
    drv->engate_pin  = pin;
}

void DRV8323_GPIO_NFAULT_Config(DRV8323_t *drv, GPIO_TypeDef *port, uint16_t pin)
{
    drv->nfault_port = port;
    drv->nfault_pin  = pin;
}

static uint32_t s_timclk1_hz;
static uint32_t s_timclk2_hz;

static void drv_cache_timer_clocks(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    s_timclk1_hz = ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) ? pclk1 * 2U : pclk1;
    s_timclk2_hz = ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) ? pclk2 * 2U : pclk2;
}

static inline uint32_t drv_get_timer_clock(TIM_HandleTypeDef *htim)
{
	drv_cache_timer_clocks();

    switch ((uint32_t)htim->Instance) {
        case TIM1_BASE: case TIM8_BASE: case TIM15_BASE:
        case TIM16_BASE: case TIM17_BASE: case TIM20_BASE:
            return s_timclk2_hz;
        default:
            return s_timclk1_hz;
    }
}

int DRV8323_TIMER_Config(DRV8323_t *drv, TIM_HandleTypeDef *timer, uint32_t freq)
{
    if (!drv || !timer) return 0;

    drv->timer    = timer;
    drv->pwm_freq = freq;

    const uint32_t timer_clock = drv_get_timer_clock(timer);

    uint32_t prescaler = 0;
    uint32_t period    = (timer_clock / (2U * freq)) - 1U;

    if (period > 0xFFFFU) {
        prescaler = period / 0xFFFFU;
        period    = (timer_clock / (2U * freq * (prescaler + 1U))) - 1U;
    }

    timer->Instance->PSC = prescaler;
    timer->Instance->ARR = period;
    drv->pwm_resolution  = period;

    return 1;
}

int DRV8323_Current_Sens_Config(DRV8323_t *drv) {
    if (!drv) return 0;

    // gain
    switch ((ADD_0x06_PRESET >> 6) & 0x3) {
    case 0: drv->gain = 5;  break;
    case 1: drv->gain = 10; break;
    case 2: drv->gain = 20; break;
    case 3: drv->gain = 40; break;
    }

    drv->v_to_current = 1.0f / (drv->gain * R_SHUNT);

    return 1;
}

/* ------------------------- Drv setup ------------------------- */
 /*
 * Call order in application:
 *   DRV8323_SPI_Config()
 *   DRV8323_GPIO_ENGATE_Config()
 *   DRV8323_GPIO_NFAULT_Config()   (optional)
 *   DRV8323_TIMER_Config()
 *   DRV8323_Current_Sens_Config()
 *   DRV8323_Init()                 <-- this function
 *   DRV8323_Enable_Gate()          <-- called separately after motor init
 */
int DRV8323_Init(DRV8323_t *drv, DRV8323_dbg_t *dbg)
{
    if (!drv || !drv->timer || !drv->spi) return 0;

    DRV8323_Stop_Pwm(drv);

    DRV8323_Enable_Gate(drv);
    HAL_Delay(10); /* t_WAKE: chờ chip khởi động nội bộ */

    DRV8323_Clear_Faults(drv);
    HAL_Delay(1);

    /* --- Write configuration registers --- */

    DRV8323_Reg_Write(drv, 0x02, ADD_0x02_PRESET);
    drv->check_spi_cfg = 2;
    HAL_Delay(1);

    DRV8323_Reg_Write(drv, 0x03, ADD_0x03_PRESET);
    drv->check_spi_cfg = 3;
    HAL_Delay(1);

    DRV8323_Reg_Write(drv, 0x04, ADD_0x04_PRESET);
    drv->check_spi_cfg = 4;
    HAL_Delay(1);

    DRV8323_Reg_Write(drv, 0x05, ADD_0x05_PRESET);
    drv->check_spi_cfg = 5;
    HAL_Delay(1);

    DRV8323_Reg_Write(drv, 0x06, ADD_0x06_PRESET);
    drv->check_spi_cfg = 6;
    HAL_Delay(10);

    DRV8323_Read_Fault1(drv, dbg);
    HAL_Delay(1);
    DRV8323_Read_Fault2(drv, dbg);

    return 1;
}

/* ------------------------- PWM control ------------------------- */

int DRV8323_Stop_Pwm(DRV8323_t *drv)
{
    if (!drv || !drv->timer) return 0;

    HAL_TIM_PWM_Stop(drv->timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(drv->timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(drv->timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(drv->timer, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Stop(drv->timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(drv->timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(drv->timer, TIM_CHANNEL_3);

    return 1;
}

int DRV8323_Start_Pwm(DRV8323_t *drv)
{
    if (!drv || !drv->timer) return 0;

    HAL_TIM_PWM_Start(drv->timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(drv->timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(drv->timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(drv->timer, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Start(drv->timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(drv->timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(drv->timer, TIM_CHANNEL_3);

    return 1;
}

void DRV8323_Set_Pwm(DRV8323_t *drv, uint32_t pwma, uint32_t pwmb, uint32_t pwmc)
{
    drv->timer->Instance->CCR1 = pwmc;
    drv->timer->Instance->CCR2 = pwmb;
    drv->timer->Instance->CCR3 = pwma;
}

/* ------------------------- Get current ------------------------- */

void DRV8323_Get_Current(DRV8323_t *drv, float *ia, float *ic)
{
//    static float ia_filtered = 0.0f;
//    static float ic_filtered = 0.0f;

    const float vshunt_a = (float)drv->adc_a * ADC_2_VOLT;
    const float vshunt_c = (float)drv->adc_c * ADC_2_VOLT;

    float ia_raw = (vshunt_a - V_OFFSET) * drv->v_to_current;
    float ic_raw = (vshunt_c - V_OFFSET) * drv->v_to_current;

//    ia_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ia_filtered + CURRENT_FILTER_ALPHA * ia_raw;
//    ic_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ic_filtered + ic_raw * CURRENT_FILTER_ALPHA;
//    *ia = ia_filtered;
//    *ic = ic_filtered;

    *ia = ia_raw;
    *ic = ic_raw;
}

/* ------------------------- SPI r/w ------------------------- */
void DRV8323_Reg_Write(DRV8323_t *drv, uint8_t addr, uint16_t data)
{
    uint16_t frame = DRV8323_SPI_WRITE
                   | ((uint16_t)(addr & 0x0FU) << DRV8323_SPI_ADDR_SHIFT)
                   | (data & DRV8323_SPI_DATA_MASK);

    uint16_t dummy;

    drv_cs_low(drv);
    drv_spi_wr_status = HAL_SPI_TransmitReceive(drv->spi, (uint8_t *)&frame, (uint8_t *)&dummy, 1, 10);
    drv_cs_high(drv);
}

uint16_t DRV8323_Reg_Read(DRV8323_t *drv, uint8_t addr)
{
    uint16_t frame = DRV8323_SPI_READ
                   | ((uint16_t)(addr & 0x0FU) << DRV8323_SPI_ADDR_SHIFT);
    uint16_t result = 0;

    drv_cs_low(drv);
    drv_spi_rd_status = HAL_SPI_TransmitReceive(drv->spi, (uint8_t *)&frame, (uint8_t *)&result, 1, 10);
    drv_cs_high(drv);

    return result & DRV8323_SPI_DATA_MASK;
}

/* ------------------------- SPI faults ------------------------- */

void DRV8323_Read_Fault1(DRV8323_t *drv, DRV8323_dbg_t *dbg)
{
    dbg->fault1 = DRV8323_Reg_Read(drv, 0x00);
}

void DRV8323_Read_Fault2(DRV8323_t *drv, DRV8323_dbg_t *dbg)
{
    dbg->fault2 = DRV8323_Reg_Read(drv, 0x01);
}

void DRV8323_Clear_Faults(DRV8323_t *drv)
{
    /* CLR_FLT bit — self-clearing */
    uint16_t ctrl = DRV8323_Reg_Read(drv, 0x02);
    DRV8323_Reg_Write(drv, 0x02, ctrl | DRV_CTRL_CLR_FLT(1U));
}

void DRV8323_Debug_Read(DRV8323_t *drv, volatile DRV8323_dbg_t *dbg)
{
    dbg->fault1     = DRV8323_Reg_Read(drv, 0x00);
    dbg->fault2     = DRV8323_Reg_Read(drv, 0x01);
    dbg->drv_ctrl   = DRV8323_Reg_Read(drv, 0x02);
    dbg->gate_hs    = DRV8323_Reg_Read(drv, 0x03);
    dbg->gate_ls    = DRV8323_Reg_Read(drv, 0x04);
    dbg->ocp_ctrl   = DRV8323_Reg_Read(drv, 0x05);
    dbg->csa_ctrl   = DRV8323_Reg_Read(drv, 0x06);
    dbg->nfault_pin = (drv->nfault_port->IDR & drv->nfault_pin) ? 1U : 0U;
    dbg->engate_pin = (drv->engate_port->IDR & drv->engate_pin) ? 1U : 0U;
}
