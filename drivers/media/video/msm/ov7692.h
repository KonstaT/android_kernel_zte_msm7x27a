/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of Code Aurora Forum, Inc. nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef OV7692_H
#define OV7692_H
#include <linux/types.h>
#include <mach/board.h>
#include <mach/camera.h>
//#define OV7692_SENSOR_ID 0x2580

extern struct ov7692_reg_t ov7692_regs;

enum ov7692_width_t {
    WORD_LEN,
    BYTE_LEN
};

struct ov7692_i2c_reg_conf {
    unsigned short waddr;
    unsigned short wdata;
    enum ov7692_width_t width;
    unsigned short mdelay_time;
};

struct ov7692_reg_t {
    struct ov7692_i2c_reg_conf const *prev_snap_reg_settings;
    uint16_t prev_snap_reg_settings_size;

    struct ov7692_i2c_reg_conf const *clk_tbl;
    uint16_t clk_tbl_sz;

    struct ov7692_i2c_reg_conf const *prevsnap_tbl;
    uint16_t prevsnap_tbl_sz;

    struct ov7692_i2c_reg_conf const *antibanding_50z_tbl;
    uint16_t antibanding_50z_tbl_sz;

    struct ov7692_i2c_reg_conf const *antibanding_60z_tbl;
    uint16_t antibanding_60z_tbl_sz;

    struct ov7692_i2c_reg_conf const *effect_off_tbl;
    uint16_t effect_off_tbl_sz;

    struct ov7692_i2c_reg_conf const *effect_mono_tbl;
    uint16_t effect_mono_tbl_sz;

    struct ov7692_i2c_reg_conf const *effect_negative_tbl;
    uint16_t effect_negative_tbl_sz;

    struct ov7692_i2c_reg_conf const *effect_sepia_tbl;
    uint16_t effect_sepia_tbl_sz;
	
    struct ov7692_i2c_reg_conf const *wb_cloudy_tbl;
    uint16_t wb_cloudy_tbl_sz;

    struct ov7692_i2c_reg_conf const *wb_daylight_tbl;
    uint16_t wb_daylight_tbl_sz;

    struct ov7692_i2c_reg_conf const *wb_flourescant_tbl;
    uint16_t wb_flourescant_tbl_sz;

    struct ov7692_i2c_reg_conf const *wb_incandescent_tbl;
    uint16_t wb_incandescent_tbl_sz;

    struct ov7692_i2c_reg_conf const *wb_auto_tbl;
    uint16_t wb_auto_tbl_sz;

    struct ov7692_i2c_reg_conf const *af_tbl;
    uint16_t af_tbl_sz;

    struct ov7692_i2c_reg_conf const **contrast_tbl;
    uint16_t const *contrast_tbl_sz;

    struct ov7692_i2c_reg_conf const **brightness_tbl;
    uint16_t const *brightness_tbl_sz;

    struct ov7692_i2c_reg_conf const **saturation_tbl;
    uint16_t const *saturation_tbl_sz;

    struct ov7692_i2c_reg_conf const **sharpness_tbl;
    uint16_t const *sharpness_tbl_sz;

    struct ov7692_i2c_reg_conf const **exposure_compensation_tbl;
    uint16_t const *exposure_compensation_tbl_sz;

    struct ov7692_i2c_reg_conf const *lens_for_outdoor_tbl;
    uint16_t const lens_for_outdoor_tbl_sz;

    struct ov7692_i2c_reg_conf const *lens_for_indoor_tbl;
    uint16_t const lens_for_indoor_tbl_sz;


};



struct reg_addr_val_pair_struct {
	uint8_t	reg_addr;
	uint8_t	reg_val;
};

enum ov7692_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum ov7692_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum ov7692_setting {
	RES_PREVIEW,
	RES_CAPTURE
};
enum ov7692_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};
#endif /* CAMSENSOR_ov7692 */


