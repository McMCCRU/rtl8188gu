/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/

/*Image2HeaderVersion: 3.1*/
#if (RTL8710B_SUPPORT == 1)
#ifndef __INC_MP_RF_HW_IMG_8710B_H
#define __INC_MP_RF_HW_IMG_8710B_H

/* Please add following compiler flags definition (#define CONFIG_XXX_DRV_DIS)
 * into driver source code to reduce code size if necessary.
 * #define CONFIG_8710B_QFN48M_SMIC_DRV_DIS
 * #define CONFIG_8710B_QFN48M_UMC_DRV_DIS
 */

#define CONFIG_8710B_QFN48M_SMIC
#ifdef CONFIG_8710B_QFN48M_SMIC_DRV_DIS
    #undef CONFIG_8710B_QFN48M_SMIC
#endif

#define CONFIG_8710B_QFN48M_UMC
#ifdef CONFIG_8710B_QFN48M_UMC_DRV_DIS
    #undef CONFIG_8710B_QFN48M_UMC
#endif

/******************************************************************************
*                           radioa.TXT
******************************************************************************/

void
odm_read_and_config_mp_8710b_radioa(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm
);
u32	odm_get_version_mp_8710b_radioa(void);

/******************************************************************************
*                           txpowertrack_pcie.TXT
******************************************************************************/

void
odm_read_and_config_mp_8710b_txpowertrack_pcie(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm
);
u32	odm_get_version_mp_8710b_txpowertrack_pcie(void);

/******************************************************************************
*                           txpowertrack_sdio.TXT
******************************************************************************/

void
odm_read_and_config_mp_8710b_txpowertrack_sdio(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm
);
u32	odm_get_version_mp_8710b_txpowertrack_sdio(void);

/******************************************************************************
*                           txpowertrack_usb.TXT
******************************************************************************/

void
odm_read_and_config_mp_8710b_txpowertrack_usb(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm
);
u32	odm_get_version_mp_8710b_txpowertrack_usb(void);

/******************************************************************************
*                           txpwr_lmt.TXT
******************************************************************************/

void
odm_read_and_config_mp_8710b_txpwr_lmt(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm
);
u32	odm_get_version_mp_8710b_txpwr_lmt(void);

/******************************************************************************
*                           txxtaltrack.TXT
******************************************************************************/

void
odm_read_and_config_mp_8710b_txxtaltrack(/* tc: Test Chip, mp: mp Chip*/
	struct	PHY_DM_STRUCT *p_dm
);
u32	odm_get_version_mp_8710b_txxtaltrack(void);

/******************************************************************************
 *                           txpowertrack_qfn48m_smic.TXT
 ******************************************************************************/

/* tc: Test Chip, mp: mp Chip*/
void
odm_read_and_config_mp_8710b_txpowertrack_qfn48m_smic(struct PHY_DM_STRUCT *dm);
u32 odm_get_version_mp_8710b_txpowertrack_qfn48m_smic(void);

/******************************************************************************
 *                           txpowertrack_qfn48m_umc.TXT
 ******************************************************************************/

/* tc: Test Chip, mp: mp Chip*/
void
odm_read_and_config_mp_8710b_txpowertrack_qfn48m_umc(struct PHY_DM_STRUCT *dm);
u32 odm_get_version_mp_8710b_txpowertrack_qfn48m_umc(void);

#endif
#endif /* end of HWIMG_SUPPORT*/
