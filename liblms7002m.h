/*
 * lms7002m compact library header file
 * Copyright (c) 2018 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 * For more information, please visit: http://xtrx.io
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#ifndef LIBLMS7002MC_H
#define LIBLMS7002MC_H

#include <stdint.h>
#include <stdbool.h>
#ifdef __linux
#include <unistd.h>
#endif

enum lms7_error_codes {
	LMSE_OK = 0,
	LMSE_OUT_OF_RANGE = 1,
};

#ifndef LMS7_EXTERN_API
#define LMS7_EXTERN_API
#endif

#define LMS7_LOGGING

enum lms7_mac_mode {
	LMS7_CH_NONE = 0,
	LMS7_CH_A = 1,
	LMS7_CH_B = 2,
	LMS7_CH_AB = LMS7_CH_A | LMS7_CH_B,
};

// State of xtsp block
struct lms7_tsp_state {
	uint16_t reg_0x0c;
};

struct lms7_filters_state {
	uint8_t rbb0_path:3;
	uint8_t rbb1_path:3;
};

struct lms7_state {
	// Global parameters
	uint32_t fref;

	// Frequent registers cache
	uint16_t reg_0x0020;
	uint8_t reg_0x0124[2]; //EN_DIR for SXX/RBB/RFE/TBB/TRF

	// RBB & TBB major states
	struct lms7_filters_state xbbst;

	// Configuration for A&B channels
	struct lms7_tsp_state rxtsp;
	struct lms7_tsp_state txtsp;
};

/* General asyncronous task */
struct lms7_async_task {
	uint8_t task_id;
	uint8_t task_subtaskid;
	uint16_t task_param16;
	uint32_t task_param32;
};

enum lms7_async_tasks {
	LMS7_TASK_CGEN_TUNE,
	LMS7_TASK_SXX_RX_TUNE,
	LMS7_TASK_SXX_TX_TUNE,
};

LMS7_EXTERN_API int lms7_spi_transact(struct lms7_state* s, uint16_t ival, uint32_t* oval);
LMS7_EXTERN_API int lms7_spi_post(struct lms7_state* s, unsigned count, const uint32_t* regs);
#ifdef LMS7_LOGGING
LMS7_EXTERN_API void lms7_log(struct lms7_state* s, const char* fmt, ...) __attribute__ ((format (printf, 2, 3)));
#else
#define lms7_log(s, fmt, ...)
#endif


// Initialize cached values
int lms7_reset(struct lms7_state* st);  // Reset internal logic
int lms7_disable(struct lms7_state* st);
int lms7_enable(struct lms7_state* st);

// MAC
int lms7_mac_set(struct lms7_state* st, enum lms7_mac_mode mode);

// CGEN functions
int lms7_cgen_disable(struct lms7_state* st);
int lms7_cgen_tune(struct lms7_state* st, unsigned outfreq, unsigned txdiv_ord);
int lms7_cgen_tune_sync(struct lms7_state* st, unsigned outfreq, unsigned txdiv_ord);


// LML functions
enum lml_mode {
	LML_NORMAL = 0,
	LML_LOOPBACK = 1,
	LML_RXLFSR = 2,
	LML_RD_FCLK = 4,
	LML_DS_HIGH = 8,
};

int lms7_lml_configure(struct lms7_state* st, bool rx_port_1,
					   unsigned txdiv, unsigned rxdiv, enum lml_mode mode);
enum lml_stream_map {
	LML_AI = 0,
	LML_AQ = 1,
	LML_BI = 2,
	LML_BQ = 3,
};
struct lml_map {
	uint8_t l[4];
};

int lms7_lml_set_map(struct lms7_state* st, struct lml_map l1m, struct lml_map l2m);

// RFE functions
enum rfe_path {
	RFE_NONE,
	RFE_LNAH,
	RFE_LNAL,
	RFE_LNAW,
	RFE_LBW,
	RFE_LBL,
};
int lms7_rfe_disable(struct lms7_state* st);
int lms7_rfe_set_path(struct lms7_state* st, enum rfe_path p, bool rfea_en, bool rfeb_en);
int lms7_rfe_set_lna(struct lms7_state* st, unsigned atten, unsigned *paout);
int lms7_rfe_set_lblna(struct lms7_state* st, unsigned attenx4, unsigned *paout);

// RBB functions
enum rbb_path {
	RBB_LBF,
	RBB_HBF,
	RBB_BYP,
	RBB_LB_LBF,
	RBB_LB_HBF,
	RBB_LB_BYP,
};
int lms7_rbb_disable(struct lms7_state* st);
int lms7_rbb_set_path(struct lms7_state* st, enum rbb_path path);
int lms7_rbb_set_pga(struct lms7_state* st, unsigned gain);
int lms7_rbb_set_bandwidth(struct lms7_state* st, unsigned bw);

// AFE functions
int lms7_afe_ctrl(struct lms7_state* st, bool rxa, bool rxb, bool txa, bool txb);

// SXX
int lms7_sxx_disable(struct lms7_state* st, bool rx);
int lms7_sxx_tune_sync(struct lms7_state* st, bool rx, unsigned lofreq, bool lochen);

// LDO
int lms7_ldo_enable(struct lms7_state* st, bool enable);

// XBUF
int lms7_xbuf_enable(struct lms7_state* st, bool bias, bool enable);

// RXTSP
//  freq
//  decim
//  tsg_const
//  read_rssi
//  dc_corr
//  iq_corr

int lms7_rxtsp_get_rssi(struct lms7_state* st, unsigned mode, uint32_t *orssi);
int lms7_rxtsp_disable(struct lms7_state* st);
int lms7_rxtsp_init(struct lms7_state* st, unsigned decim_ord);
int lms7_rxtsp_cmix(struct lms7_state* st, int32_t freq);
int lms7_rxtsp_tsg_const(struct lms7_state* st, int16_t vi, int16_t vq);
int lms7_rxtsp_tsg_tone(struct lms7_state* st, bool fs, bool div4);
int lms7_rxtsp_dc_corr(struct lms7_state* st, unsigned wnd);

// TXTSP
int lms7_txtsp_disable(struct lms7_state* st);
int lms7_txtsp_init(struct lms7_state* st, unsigned interp_ord);
int lms7_txtsp_cmix(struct lms7_state* st, int32_t freq);
int lms7_txtsp_tsg_const(struct lms7_state* st, int16_t vi, int16_t vq);
int lms7_txtsp_tsg_tone(struct lms7_state* st, bool fs, bool div4);

// TBB
enum tbb_path {
	TBB_BYP,
	TBB_S5,
	TBB_LAD,
	TBB_LADS5,
	TBB_HBF,
};

int lms7_tbb_disable(struct lms7_state* st);
int lms7_tbb_set_path(struct lms7_state* st, enum tbb_path path);
int lms7_tbb_set_bandwidth(struct lms7_state* st, unsigned bw);

// TRF
int lms7_trf_disable(struct lms7_state* st);
int lms7_trf_enable(struct lms7_state* st, bool cha, bool chb);
int lms7_trf_set_pad(struct lms7_state* st, unsigned atten);
int lms7_trf_set_path(struct lms7_state* st, unsigned band);

// DC
int lms7_dc_init(struct lms7_state* st, bool rxaen, bool rxben, bool txaen, bool txben);
int lms7_dc_start(struct lms7_state* st, bool rxa, bool rxb, bool txa, bool txb);

// Helper functions

struct vco_nint_nfrac {
	unsigned nint;
	unsigned frac;
};
struct vco_nint_nfrac lms7_pll_calc(unsigned fref, unsigned vco);

// Calibration API
enum vco_cmp {
	VCO_CMP_LOW = 0,
	VCO_CMP_FAIL = 1,
	VCO_CMP_OK = 2,
	VCO_CMP_HIGH = 3,
};

int lms7_sxx_get_comp(struct lms7_state* st);
int lms7_cgen_get_comp(struct lms7_state* st);


int lms7_cgen_find_cap(struct lms7_state* st, unsigned start, uint8_t* phi, uint8_t* plo);


#define REG_COUNT(x) (sizeof(x) / sizeof(x[0]))

enum cgen_vco_params {
	CGEN_VCO_MIN = 2000000000U,
	CGEN_VCO_MAX = 2700000000U,
	CGEN_VCO_MID = CGEN_VCO_MIN / 2 + CGEN_VCO_MAX / 2,
	CGEN_VCO_RANGE = CGEN_VCO_MAX - CGEN_VCO_MIN,
};

enum {
	VCAL_LOW = 8,
	VCAL_NORM = 64,
	VCAL_HIGH = 8,
};

int lms7_cal_rxdc(struct lms7_state* st);

#endif //LIBLMS7002MC_H
