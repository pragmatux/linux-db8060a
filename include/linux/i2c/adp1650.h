/*
 * ADP1650 LED Flash Driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __LINUX_I2C_ADP1650_H
#define __LINUX_I2C_ADP1650_H

/*
 * ADP1650 Registers
 */
#define ADP1650_REG_VERSION		0x00
#define ADP1650_REG_TIMER_IOCFG		0x02
#define ADP1650_REG_CURRENT_SET		0x03
#define ADP1650_REG_OUTPUT_MODE		0x04
#define ADP1650_REG_FAULT		0x05
#define ADP1650_REG_CONTROL		0x06
#define ADP1650_REG_AD_MODE		0x07
#define ADP1650_REG_ADC			0x08
#define ADP1650_REG_BATT_LOW		0x09

/* ADP1650_REG_TIMER_IOCFG Bits and Masks */
#define ADP1650_IOCFG_IO2_HIGH_IMP	(0 << 6) /* High Impedance */
#define ADP1650_IOCFG_IO2_IND_LED	(1 << 6) /* Indicator LED */
#define ADP1650_IOCFG_IO2_TXMASK2	(2 << 6) /* TxMASK2 operation mode */
#define ADP1650_IOCFG_IO2_AIN		(3 << 6) /* ADC analog input */
#define ADP1650_IOCFG_IO1_HIGH_IMP	(0 << 4) /* High Impedance */
#define ADP1650_IOCFG_IO1_TORCH		(1 << 4) /* Torch mode */
#define ADP1650_IOCFG_IO1_TXMASK1	(2 << 4) /* TxMASK1 operation mode */
#define ADP1650_FL_TIMER_ms(x)		((((x) - 100) / 100) & 0xF) /* Timer */

/* ADP1650_REG_CURRENT_SET Bits and Masks  */
#define ADP1650_I_FL_mA(x)		((((x) - 300) / 50) << 3)
#define ADP1650_I_TOR_mA(x)		((((x) - 25) / 25) & 0x7)

/* ADP1650_REG_OUTPUT_MODE Bits and Masks  */
#define ADP1650_IL_PEAK_1A75		(0 << 6)
#define ADP1650_IL_PEAK_2A25		(1 << 6)
#define ADP1650_IL_PEAK_2A75		(2 << 6)
#define ADP1650_IL_PEAK_3A00		(3 << 6)
#define ADP1650_STR_LV_EDGE		(0 << 5)
#define ADP1650_STR_LV_LEVEL		(1 << 5)
#define ADP1650_FREQ_FB_EN		(1 << 4)
#define ADP1650_OUTPUT_EN		(1 << 3)
#define ADP1650_STR_MODE_SW		(0 << 2)
#define ADP1650_STR_MODE_HW		(1 << 2)
#define ADP1650_STR_MODE_STBY		(0 << 0)
#define ADP1650_LED_MODE_VOUT		(1 << 0)
#define ADP1650_LED_MODE_ASSIST_LIGHT	(2 << 0)
#define ADP1650_LED_MODE_FLASH		(3 << 0)

/* ADP1650_REG_FAULT Bits and Masks  */
#define ADP1650_FL_OVP			(1 << 7)
#define ADP1650_FL_SC			(1 << 6)
#define ADP1650_FL_OT			(1 << 5)
#define ADP1650_FL_TO			(1 << 4)
#define ADP1650_FL_TX1			(1 << 3)
#define ADP1650_FL_IO2			(1 << 2)
#define ADP1650_FL_IL			(1 << 1)
#define ADP1650_FL_IDC			(1 << 0)

/* ADP1650_REG_CONTROL Bits and Masks  */
#define ADP1650_I_TX2_mA(x)		((((x) - 100) / 50) << 4)
#define ADP1650_I_TX1_mA(x)		((((x) - 100) / 50) & 0xF)

/* ADP1650_REG_AD_MODE Bits and Masks  */
#define ADP1650_DYN_OVP_EN			(1 << 7)
#define ADP1650_SW_LO_1MHz5			(1 << 6)
#define ADP1650_STR_POL_ACTIVE_HIGH		(1 << 5)
#define ADP1650_I_ILED_2mA75			(0 << 4)
#define ADP1650_I_ILED_5mA50			(1 << 4)
#define ADP1650_I_ILED_8mA25			(2 << 4)
#define ADP1650_I_ILED_11mA00			(3 << 4)
#define ADP1650_IL_DC_1A50			(0 << 1)
#define ADP1650_IL_DC_1A75			(1 << 1)
#define ADP1650_IL_DC_2A00			(2 << 1)
#define ADP1650_IL_DC_2A25			(3 << 1)
#define ADP1650_IL_DC_EN			(1 << 0)

/* ADP1650_REG_ADC Bits and Masks  */
#define ADP1650_FL_VB_LO			(1 << 6)
#define ADP1650_ADC_VAL(x)			(((x) & 0x3C) >> 2)
#define ADP1650_ADC_DIS				(0 << 0)
#define ADP1650_ADC_LED_VF			(1 << 0)
#define ADP1650_ADC_DIE_TEMP			(2 << 0)
#define ADP1650_ADC_EXT_VOLT			(3 << 0)

/* ADP1650_REG_BATT_LOW Bits and Masks  */
#define ADP1650_CL_SOFT_EN			(1 << 7)
#define ADP1650_I_VB_LO_mA(x)			((((x) - 300) / 50) << 3)
#define ADP1650_V_VB_LO_DIS			(0 << 0)
#define ADP1650_V_VB_LO_3V30			(1 << 0)
#define ADP1650_V_VB_LO_3V35			(2 << 0)
#define ADP1650_V_VB_LO_3V40			(3 << 0)
#define ADP1650_V_VB_LO_3V45			(4 << 0)
#define ADP1650_V_VB_LO_3V50			(5 << 0)
#define ADP1650_V_VB_LO_3V55			(6 << 0)
#define ADP1650_V_VB_LO_3V60			(7 << 0)

/*
 * /sys/class/leds/adp1650/brightness values / mode steering
 */

#define FL_MODE_OFF			0 /* OFF */
#define FL_MODE_TORCH_25mA		1 /* SW trigged TORCH to FLASH */
#define FL_MODE_TORCH_50mA		2 /* TORCH Intensity XmA */
#define FL_MODE_TORCH_75mA		3
#define FL_MODE_TORCH_100mA		4
#define FL_MODE_TORCH_125mA		5
#define FL_MODE_TORCH_150mA		6
#define FL_MODE_TORCH_175mA		7
#define FL_MODE_TORCH_200mA		8
#define FL_MODE_TORCH_TRIG_EXT_25mA	9 /* HW/IO trigged TORCH to FLASH */
#define FL_MODE_TORCH_TRIG_EXT_50mA	10/* TORCH Intensity XmA */
#define FL_MODE_TORCH_TRIG_EXT_75mA	11
#define FL_MODE_TORCH_TRIG_EXT_100mA	12
#define FL_MODE_TORCH_TRIG_EXT_125mA	13
#define FL_MODE_TORCH_TRIG_EXT_150mA	14
#define FL_MODE_TORCH_TRIG_EXT_175mA	15
#define FL_MODE_TORCH_TRIG_EXT_200mA	16
#define FL_MODE_FLASH			254 /* SW triggered FLASH */
#define FL_MODE_FLASH_TRIG_EXT		255 /* HW/Strobe trigged FLASH */

struct i2c_client; /* forward declaration */

struct adp1650_leds_platform_data {
	unsigned char timer_iocfg;	/* See ADP1650_REG_TIMER_IOCFG Bits */
	unsigned char current_set;	/* See ADP1650_REG_CURRENT_SET Bits */
	unsigned char output_mode;	/* See ADP1650_REG_OUTPUT_MODE Bits */
	unsigned char control;		/* See ADP1650_REG_CONTROL Bits */
	unsigned char ad_mode;		/* See ADP1650_REG_AD_MODE Bits */
	unsigned char batt_low;		/* See ADP1650_REG_BATT_LOW Bits */

	/* system specific GPIO to control the ADP1650_EN pin,
	 * if not used set to -1
	 */
	int gpio_enable;

	/* system specific setup callback */
	int	(*setup)(struct i2c_client *client,
			 unsigned state);
};

#endif /* __LINUX_I2C_ADP1650_H */
