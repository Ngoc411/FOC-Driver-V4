/*  * bldc_encoder.c
 *
 *  Created on: Jan 9, 2026
 *      Author: Ngoc
 */

#include "MT6816.h"
#include "FOC_utils.h"

// Register addresses (separate from struct)
const uint8_t reg_addrs[2] = {0x83, 0x84};

// Static buffer for DMA
static uint8_t tx_buf[2] = {0x83, 0x00};

// ---------------- Inline Helper Functions ----------------
static inline bool checkEvenParity(uint16_t data) {
    // Use XOR to count bits faster
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return !(data & 1); // Even parity
}

static inline float wrap_pm_180(float deg) {
	float wraped_deg = deg - (360.0f * floorf((deg + 180.0f) / 360.0f));
	return wraped_deg;
}

// ---------------- Public Functions ----------------
void MT6816_Config(MT6816_t *mt6816, GPIO_TypeDef *CS_Port, uint16_t CS_Pin, SPI_HandleTypeDef *spi) {
	mt6816->rawData = 0;
	mt6816->rawAngle = 0;
	mt6816->magCheck = 0;
	mt6816->parityCheck = 0;

	mt6816->prevMechTheta = 0.0f;
	mt6816->mechTheta = 0.0f;
	mt6816->vel = 0.0f;

	mt6816->rx_8Bit[0] = 0;
	mt6816->rx_8Bit[1] = 0;
	mt6816->complete_16Bit[0] = 0;
	mt6816->complete_16Bit[1] = 0;
	mt6816->read_flag = 0;
	mt6816->check = 0;
	mt6816->initialized = 0;
	mt6816->validCount = 0;
	mt6816->parityErrorCount = 0;
	mt6816->spikeRejectCount = 0;
	mt6816->spiBusyCount = 0;
	mt6816->spiErrorCount = 0;
	mt6816->lastUpdateMs = 0;

	mt6816->CS_Port = CS_Port;
	mt6816->CS_Pin = CS_Pin;
	mt6816->spi = spi;

	mt6816->multiturnAngleRound = 0;
}

inline void MT6816_Select(MT6816_t *mt6816) {
	mt6816->CS_Port->BSRR = (mt6816->CS_Pin << 16); // GPIO_PIN_RESET
	mt6816->cs_status = 0;
}

inline void MT6816_DeSelect(MT6816_t *mt6816) {
	mt6816->CS_Port->BSRR = mt6816->CS_Pin; // GPIO_PIN_SET
	mt6816->cs_status = 1;
}

/* -------------------------------- utils -------------------------------- */

void MT6816_Start(MT6816_t *mt6816) {
	MT6816_DeSelect(mt6816);
	HAL_Delay(20);
	MT6816_Spi_Transmit_Receive_Data_8Bit(mt6816, reg_addrs[0]);
//	encoder.check = 1;
}

void MT6816_Get_RPM(MT6816_t *mt6816, FOC_t *foc, float dt_us) {
    // Optimized angle wrap-around
    float dtheta = wrap_pm_180(mt6816->mechTheta - mt6816->prevMechTheta);

    mt6816->prevMechTheta = mt6816->mechTheta;
    mt6816->dtheta = dtheta;

    // Calculate instantaneous RPM (optimized constants)
    const float dt_sec = dt_us * 1e-6f;  // Convert microseconds to seconds
    const float rpm_conversion = 60.0f / 360.0f;  // Degrees/sec to RPM (1 rev = 360°, 1 min = 60 sec)
    float rpm_instant = (dtheta / dt_sec) * rpm_conversion;

    // Two-stage spike rejection
#ifdef USE_RPM_SPIKE_REJECTION
    float rpm_delta = rpm_instant - mt6816->prevRPM;
    float abs_delta = fabsf(rpm_delta);

    if (abs_delta > MAX_RPM_JUMP) {
        float limited_delta = copysignf(fminf(abs_delta * 0.5f, MAX_RPM_JUMP), rpm_delta);
        rpm_instant = mt6816->prevRPM + limited_delta;
    }
#endif

    // IIR low-pass filter
#ifdef USE_RPM_FILTER
    mt6816->vel = mt6816->vel * (1.0f - ANGLE_FILTER_ALPHA) + rpm_instant * ANGLE_FILTER_ALPHA;
#else
    mt6816->vel = rpm_instant;
#endif

    // Very low RPM clamping
    if (fabsf(mt6816->vel) < 0.1f) {
        mt6816->vel = 0.0f;
    }

    // Update state for next iteration
#if defined(USE_RPM_SPIKE_REJECTION) || defined(USE_RPM_FILTER)
    mt6816->prevRPM = rpm_instant;
#endif

    // update foc
    foc->encd_speed = mt6816->vel;
}

/* -------------------------------- 8-bit version -------------------------------- */
void MT6816_Spi_Transmit_Receive_Data_8Bit(MT6816_t *mt6816, uint8_t reg) {
    tx_buf[0] = reg;

    if (mt6816->check || HAL_SPI_GetState(mt6816->spi) != HAL_SPI_STATE_READY) {
    	mt6816->spiBusyCount++;
    	return;
    }

    MT6816_Select(mt6816);
    if (HAL_SPI_TransmitReceive_DMA(mt6816->spi, tx_buf, mt6816->rx_8Bit, 2) != HAL_OK) {
    	MT6816_DeSelect(mt6816);
    	mt6816->spiErrorCount++;
    	mt6816->check = 0;
    	return;
    }

    mt6816->check = 1;
//    mt6816->check ++;
}

void MT6816_Update_Angle_8Bit(MT6816_t *mt6816, FOC_t *foc) {
	// Combine 2 bytes
    mt6816->rawData = ((uint16_t)(mt6816->complete_16Bit[0]) << 8) | (mt6816->complete_16Bit[1]);

    mt6816->parityCheck = checkEvenParity(mt6816->rawData);

	// If parity fails → keep previous filtered angle
	if (!mt6816->parityCheck) {
		mt6816->parityErrorCount++;
		return;
	}

	mt6816->rawAngle = mt6816->rawData >> 2;  // 14-bit angle
	mt6816->magCheck = (bool)(mt6816->rawData & 0x02);

	// Convert raw angle to mechanical degrees
	float rawTheta = mt6816->rawAngle * (360.0f / MT6816_RESOLUTION);

	if (!mt6816->initialized) {
		mt6816->prevRawTheta = rawTheta;
		mt6816->mechTheta = rawTheta;
		mt6816->prevMechTheta = rawTheta;
		mt6816->spikeCounter = 0;
		mt6816->initialized = 1;
	}

	/* ============================ 1. Spike / jump rejection ============================ */
	float rawDiff = wrap_pm_180(rawTheta - mt6816->prevRawTheta);

	if (fabsf(rawDiff) > MAX_ANGLE_JUMP_DEG) {
		if (++(mt6816->spikeCounter) < SPIKE_REJECT_COUNT) {
			// keep the previous states - do nothing
			mt6816->spikeRejectCount++;
			return;
		}
		mt6816->spikeCounter = 0;
	} else {
		mt6816->spikeCounter = 0;
	}

	mt6816->prevRawTheta = rawTheta;

	/* ============================ 2. IIR low-pass (wrap-aware) ============================ */
	float filtDiff = wrap_pm_180(rawTheta - mt6816->mechTheta);
	mt6816->mechTheta += ANGLE_FILTER_ALPHA * filtDiff;

	// Normalize to [0, 360)
	if (mt6816->mechTheta >= 360.0f)
		mt6816->mechTheta -= 360.0f;
	else if (mt6816->mechTheta < 0.0f)
		mt6816->mechTheta += 360.0f;

	/* ============================ Update FOC ============================ */
	foc->m_rad_raw = DEG_TO_RAD(mt6816->mechTheta);
	mt6816->validCount++;
	mt6816->lastUpdateMs = HAL_GetTick();
}


void MT6816_Get_Multiturn_Degree(MT6816_t *mt6816, FOC_t *foc) { // get the multiturn angle
    const float m_current_angle = mt6816->mechTheta;
	float angle_dif = (m_current_angle - mt6816->prevMultiturnAngle);

	if (angle_dif< -180) {
		mt6816->multiturnAngleRound++;
	}
	else if (angle_dif> 180) {
		mt6816->multiturnAngleRound--;
	}
	else if (mt6816->multiturnAngleRound != 0 && foc->check_mul_round) {
		mt6816->multiturnAngleRound = 0;
		foc->check_mul_round = 0;
	}
	float out_deg = (m_current_angle + mt6816->multiturnAngleRound * 360.0 - RAD_TO_DEG(foc->m_rad_offset));
    mt6816->multiturnAngle = ((1.0f - ACTUAL_ANGLE_FILTER_ALPHA) * mt6816->multiturnAngle + ACTUAL_ANGLE_FILTER_ALPHA * out_deg);
	foc->mutil_turn_deg = mt6816->multiturnAngle;
    mt6816->prevMultiturnAngle = m_current_angle;
}
