/****************************************************************************
 *
 * Copyright 2022 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
#ifndef __DRIVERS_AUDIO_UB6470SCRIPTS_H
#define __DRIVERS_AUDIO_UB6470SCRIPTS_H

#include <tinyara/config.h>

#ifdef CONFIG_AUDIO_UB6470

/****************************************************************************
 * Included Files
 ****************************************************************************/


typedef enum {
        UB6470_REG_D_1BYTE,
        UB6470_REG_D_4BYTE,
        UB6470_REG_DATA_TYPE_MAX
} UB6470_REG_DATA_TYPE;

/* TYPEDEFS */
typedef struct {
	char /* UB6470_REG_DATA_TYPE */ type;
	uint8_t addr;  /* UB6470_REG_1BYTE_MODE addr or UB6470_REG_4BYTE_MODE addr */
	uint8_t val[4];
	unsigned int delay;
} t_codec_init_script_entry;

t_codec_init_script_entry codec_stop_script[] = {
	{UB6470_REG_D_1BYTE, 0x2A, {0x00,}, 0}
};

t_codec_init_script_entry codec_initial_script[] = { /* refer to regs_default.h file for ub647x */

	//REG_IvDisEn  = 0x0000


	//Main Page Setting

	//PLL Setting
	{UB6470_REG_D_1BYTE, 0x04, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x05, {0x3F,}, 0},
	{UB6470_REG_D_1BYTE, 0x03, {0x01,}, 0},

	//Power Down On/Off
	{UB6470_REG_D_1BYTE, 0x01, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x01, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x01, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x01, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x01, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x01, {0x00,}, 0},

//DRC Filter Disable Setting
	{UB6470_REG_D_1BYTE, 0x2F, {0x00,}, 0},

//EQ Disable Setting
	{UB6470_REG_D_1BYTE, 0x25, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x26, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x27, {0x00,}, 0},
	{UB6470_REG_D_1BYTE, 0x28, {0x00,}, 0},

//Dynamic Boost Disable
	{UB6470_REG_D_1BYTE, 0x6B, {0x00,}, 0},

//Audio IF Page Setting

//SDIO Format1 Setting
	{UB6470_REG_D_1BYTE, 0x10, {0xD0,}, 0},

//SDIO Format2 Setting
	{UB6470_REG_D_1BYTE, 0x11, {0x50,}, 0},

//SDO Enable Setting
	{UB6470_REG_D_1BYTE, 0x13, {0x00,}, 0},

//Mixer Page Setting

//Tuning need by h/w lab
//Input L to OutPut L Mixer
{UB6470_REG_D_1BYTE, 0x20, {0xCF,}, 0},

//Input R to OutPut L Mixer = Mute
{UB6470_REG_D_1BYTE, 0x21, {0x00,}, 0},

//Input L to OutPut R Mixer = Mute
{UB6470_REG_D_1BYTE, 0x22, {0x00,}, 0},

//Input R to OutPut R Mixer
{UB6470_REG_D_1BYTE, 0x23, {0xCF,}, 0},

//Mixer Polarity Setting
{UB6470_REG_D_1BYTE, 0x24, {0x00,}, 0},

//Speaker Left Volume = 0.000
{UB6470_REG_D_1BYTE, 0x2C, {0xCF,}, 0},

//Speaker Right Volume = 0.000
{UB6470_REG_D_1BYTE, 0x2D, {0xCF,}, 0},

//Headphone Left Volume = -6.000
{UB6470_REG_D_1BYTE, 0x6E, {0xC3,}, 0},

//Headphone Right Volume = -6.000
{UB6470_REG_D_1BYTE, 0x6F, {0xC3,}, 0},


//Master/Fine Volume = 5.0000
{UB6470_REG_D_1BYTE, 0x2E, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x2A, {0xCC,}, 0},
{UB6470_REG_D_1BYTE, 0x2B, {0x00,}, 0},


// Errro Monitoring Auto Clear
{UB6470_REG_D_1BYTE, 0x0E, {0x00,}, 0},

//Output Control Page Setting

//PWM Output Mapping
{UB6470_REG_D_1BYTE, 0x41, {0xE4,}, 0},


//PWM SP Setting
{UB6470_REG_D_1BYTE, 0x44, {0x20,}, 0},


//PWM SP Min Pulse
{UB6470_REG_D_1BYTE, 0x45, {0x03,}, 0},


//PWM Left Initial Pulse
{UB6470_REG_D_1BYTE, 0x46, {0x13,}, 0},

//PWM Left BD Mode Offset
{UB6470_REG_D_1BYTE, 0x47, {0x0A,}, 0},

//PWM Right Initial Pulse
{UB6470_REG_D_1BYTE, 0x48, {0x1F,}, 0},

//PWM Right BD Mode Offset
{UB6470_REG_D_1BYTE, 0x49, {0x03,}, 0},

//PWM BD Mode Offset End
{UB6470_REG_D_1BYTE, 0x4A, {0x06,}, 0},


//Power Stage Recovery Time
{UB6470_REG_D_1BYTE, 0x51, {0xC0,}, 0},


//Power Stage Error Count Mode
{UB6470_REG_D_1BYTE, 0x52, {0x10,}, 0},


//Headphone Mute Control
{UB6470_REG_D_1BYTE, 0x4E, {0x11,}, 0},

//Headphone Phase Control
{UB6470_REG_D_1BYTE, 0x5D, {0x88,}, 0},


//External Error Effectr
{UB6470_REG_D_1BYTE, 0x53, {0x01,}, 0},


//Mute When Power Stage Error
{UB6470_REG_D_1BYTE, 0x50, {0x00,}, 0},

//Power Stage Monitor1
{UB6470_REG_D_1BYTE, 0x5E, {0x00,}, 0},

//Power Stage Monitor2
{UB6470_REG_D_1BYTE, 0x5F, {0x00,}, 0},

//Left Clipping Level = 24.0dB
{UB6470_REG_D_1BYTE, 0x70, {0xFF,}, 0},

//Right Clipping Level = 24.0dB
{UB6470_REG_D_1BYTE, 0x71, {0xFF,}, 0},

//PWM DC Cut Filter Setting
{UB6470_REG_D_1BYTE, 0x42, {0x01,}, 0},

//PWM SNR/THD Setting
{UB6470_REG_D_1BYTE, 0x43, {0x9C,}, 0},
{UB6470_REG_D_1BYTE, 0x56, {0x50,}, 0},
{UB6470_REG_D_1BYTE, 0x58, {0x10,}, 0},
{UB6470_REG_D_1BYTE, 0x50, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x7F, {0x97,}, 0},

//MOCS Page Setting

//MOCS Setting(Offset, Min, Gain)
{UB6470_REG_D_1BYTE, 0x00, {0x01,}, 0},
{UB6470_REG_D_4BYTE, 0xCC, {0x00, 0x10, 0x00, 0x00}, 0},
{UB6470_REG_D_4BYTE, 0xCD, {0x00, 0x28, 0x00, 0x00}, 0},
{UB6470_REG_D_4BYTE, 0xCE, {0x00, 0x00, 0x00, 0x80}, 0},
{UB6470_REG_D_4BYTE, 0xCF, {0x00, 0x04, 0x00, 0x00}, 0},
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},


//MOCS Hold Delay
{UB6470_REG_D_1BYTE, 0x75, {0x20,}, 0},


//MOCS Output Control
{UB6470_REG_D_1BYTE, 0x74, {0x00,}, 0},


//DRC Page Setting

//DRC Enable Setting
{UB6470_REG_D_1BYTE, 0x30, {0x00,}, 0},

//DRC1 Threshold = 2.5dB
{UB6470_REG_D_1BYTE, 0x31, {0xD4,}, 0},

//DRC2 Threshold = 0.0dB
{UB6470_REG_D_1BYTE, 0x32, {0xCF,}, 0},



//Compress Enable Setting
{UB6470_REG_D_1BYTE, 0x33, {0x00,}, 0},

//Cmp1 Threshold = -3.0dB
{UB6470_REG_D_1BYTE, 0x34, {0xC9,}, 0},

//Cmp2 Threshold = -3.0dB
{UB6470_REG_D_1BYTE, 0x35, {0xC9,}, 0},

//Cmp1 Slop = 0.500
{UB6470_REG_D_1BYTE, 0x36, {0x40,}, 0},

//Cmp2 Slop = 1.000
{UB6470_REG_D_1BYTE, 0x37, {0x80,}, 0},



//Left DRC1 Mixer Level = 0.0dB
{UB6470_REG_D_1BYTE, 0x38, {0xCF,}, 0},

//Left DRC2 Mixer Level = Mute
{UB6470_REG_D_1BYTE, 0x39, {0x00,}, 0},

//Right DRC1 Mixer Level = 0.0dB
{UB6470_REG_D_1BYTE, 0x3A, {0xCF,}, 0},

//Right DRC2 Mixer Level = Mute
{UB6470_REG_D_1BYTE, 0x3B, {0x00,}, 0},

//DRC1 Attack/Release Setting
{UB6470_REG_D_4BYTE, 0xF8, {0x00, 0x00, 0x04, 0x18}, 0},
{UB6470_REG_D_4BYTE, 0xF9, {0x00, 0x3F, 0xFB, 0xE7}, 0},
{UB6470_REG_D_4BYTE, 0xFA, {0x00, 0x3F, 0xF4, 0x2C}, 0},
{UB6470_REG_D_4BYTE, 0xFB, {0x00, 0x40, 0x00, 0x40}, 0},


//DRC2 Attack/Release Setting
{UB6470_REG_D_4BYTE, 0xFC, {0x00, 0x00, 0x04, 0x18}, 0},
{UB6470_REG_D_4BYTE, 0xFD, {0x00, 0x3F, 0xFB, 0xE7}, 0},
{UB6470_REG_D_4BYTE, 0xFE, {0x00, 0x3F, 0x5C, 0x28}, 0},
{UB6470_REG_D_4BYTE, 0xFF, {0x00, 0x40, 0x00, 0x40}, 0},


//Page 1 Setting
{UB6470_REG_D_1BYTE, 0x00, {0x01,}, 0},
//EQ Filter Checksum Setting
{UB6470_REG_D_4BYTE, 0xD0, {0x00, 0x0F, 0x85, 0x9E}, 0},
//Page 0 Setting
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},




//Page 1 Setting
{UB6470_REG_D_1BYTE, 0x00, {0x01,}, 0},
//DRC Filter Checksum Setting
{UB6470_REG_D_4BYTE, 0xD1, {0x00, 0x00, 0x00, 0x00}, 0},
//Page 0 Setting
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},




//EQ Mode
{UB6470_REG_D_1BYTE, 0x67, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x68, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x69, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x6A, {0x00,}, 0},

//Dynamic Boost Page

//Boost Threshold = -3.0
{UB6470_REG_D_1BYTE, 0x6C, {0xC9,}, 0},

//Boost Slope = 0.500
{UB6470_REG_D_1BYTE, 0x6D, {0x40,}, 0},


//Dynamic Boost Attack/Release Setting
{UB6470_REG_D_4BYTE, 0xF6, {0x00, 0x3F, 0x5C, 0x28}, 0},
{UB6470_REG_D_4BYTE, 0xF7, {0x00, 0x40, 0x00, 0x40}, 0},

//Boost Gain = 18.0
{UB6470_REG_D_4BYTE, 0xF3, {0x00, 0x7F, 0xFF, 0xFF}, 0},

//Response Gain = -40

//Delay
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x00, {0x00,}, 0},

//Boost Enable = OFF, EQ Adding = ON, EQ No = 1
{UB6470_REG_D_1BYTE, 0x6B, {0x40,}, 0},

//DRC Filter Enable Setting
{UB6470_REG_D_1BYTE, 0x2F, {0x00,}, 0},
//EQ Enable Setting
{UB6470_REG_D_1BYTE, 0x25, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x26, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x27, {0x00,}, 0},
{UB6470_REG_D_1BYTE, 0x28, {0x00,}, 0},

//PWM Speaker/Headphone Select
{UB6470_REG_D_1BYTE, 0x57, {0x00,}, 0},


//PWM Output Enable On/Off - default stereo
{UB6470_REG_D_1BYTE, 0x40, {0x03,}, 0},


//DSP Mute On/Off
{UB6470_REG_D_1BYTE, 0x29, {0x00,}, 0},

//Monitoring Register Enable
{UB6470_REG_D_1BYTE, 0x0E, {0x01,}, 0}
//End identifier
//{END_IDENTIFIER, 0x00}

};

t_codec_init_script_entry codec_init_mute_on_script[] = {
	{UB6470_REG_D_1BYTE, 0x2A, {0x00,}, 0}
};

t_codec_init_script_entry codec_set_master_volume_script[] = {
	{UB6470_REG_D_1BYTE, 0x2A, {0x03,}, 0},
	{UB6470_REG_D_1BYTE, 0x2A, {0xFF,}, 0}
};


#endif	/* CONFIG_AUDIO_UB6470 */
#endif	/* __DRIVERS_AUDIO_UB6470SCRIPTS_H */
