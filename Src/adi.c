/***********************
*
*/

#include "adi.h"

AD7792_HandleTypeDef adi1;

void ADI_Init(void) {
	//
	adi1.conf &= ( ~AD7792_CONF_GAIN(0xFF) | ~AD7792_CONF_CHAN(0xFF) | ~AD7792_CONF_UNIPOLAR | AD7792_CONF_BUF);
	adi1.conf |= ( AD7792_CONF_GAIN(AD7792_GAIN_1) | AD7792_CONF_CHAN(AD7792_CH_AIN2P_AIN2M) | AD7792_CONF_UNIPOLAR | AD7792_CONF_BUF);
	
	adi1.mode &= ~AD7792_MODE_RATE(0xF);
	adi1.mode |= AD7792_MODE_RATE(AD7702_RATE_1_120ms);
	
	adi1.io |= ( AD7792_IEXCDIR(AD7792_DIR_IEXC1_IOUT1_IEXC2_IOUT2) | AD7792_IEXCEN(AD7792_EN_IXCEN_210uA) );
	
	//adi1.offset[AD7792_CH_AIN2P_AIN2M] = AD7792_GetRegisterValue(AD7792_REG_OFFSET, 2, 1);
	adi1.offset[AD7792_CH_AIN2P_AIN2M] = 0x8000;
	//adi1.fullscale[AD7792_CH_AIN2P_AIN2M] = AD7792_GetRegisterValue(AD7792_REG_FULLSCALE, 2, 1);
	adi1.fullscale[AD7792_CH_AIN2P_AIN2M] = 0x54A3;
	
	adi1.cs.gpio = GPIOB;
	adi1.cs.pin = GPIO_PIN_12;
	adi1.rdy.gpio = GPIOC;
	adi1.rdy.pin = GPIO_PIN_6;
	
	AD7792_conf(&adi1, reg_all);
	
	/*if(AD7792_Init() == 1) {
		//
		AD7792_conf(&adi1, reg_all);
	}*/
}

uint32_t rec_filter(uint32_t data, uint8_t Nb, uint8_t k) {
  static uint32_t y = 0;
  static uint64_t z = 0;
  z += (data - y);
  return y = (Nb * z) >> k;

//	static uint32_t res;
//	res += data;
//	res /= 2;
//	return res;
}



