/** @file
*
*  Copyright (c) 2015, Linaro Ltd. All rights reserved.
*  Copyright (c) 2015, Hisilicon Ltd. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef __HI6220_REGS_PERI_H__
#define __HI6220_REGS_PERI_H__

#define HI6220_PERI_BASE                0xf7030000
#define HI6220_PMU_SSI_BASE             0xf8000000

#define PMUSSI_REG_EX(reg_addr) (((reg_addr) << 2) + HI6220_PMU_SSI_BASE)

#define SC_PERIPH_RSTEN3                (HI6220_PERI_BASE + 0x330)
#define SC_PERIPH_RSTDIS0               (HI6220_PERI_BASE + 0x304)
#define SC_PERIPH_RSTDIS3               (HI6220_PERI_BASE + 0x334)
#define SC_PERIPH_RSTSTAT0              (HI6220_PERI_BASE + 0x308)
#define SC_PERIPH_RSTSTAT3              (HI6220_PERI_BASE + 0x338)

#define SC_PMIC_CLK19M2_600_586_EN      0xfe
#define SC_PMIC_WLAN_CLK                0x1c
#define SC_PMIC_WLAN_CLK_ENABLE         0x40


#define PERI_RST0_MMC0                  (1 << 0)
#define PERI_RST0_MMC1                  (1 << 1)
#define PERI_RST0_MMC2                  (1 << 2)
#define PERI_RST0_NANDC                 (1 << 3)
#define PERI_RST0_USBOTG_BUS            (1 << 4)
#define PERI_RST0_POR_PICOPHY           (1 << 5)
#define PERI_RST0_USBOTG                (1 << 6)
#define PERI_RST0_USBOTG_32K            (1 << 7)

/* SC_PERIPH_RSTEN3/RSTDIS3/RSTSTAT3 */
#define PERIPH_RST3_CSSYS               (1 << 0)
#define PERIPH_RST3_I2C0                (1 << 1)
#define PERIPH_RST3_I2C1                (1 << 2)
#define PERIPH_RST3_I2C2                (1 << 3)
#define PERIPH_RST3_I2C3                (1 << 4)
#define PERIPH_RST3_UART1               (1 << 5)
#define PERIPH_RST3_UART2               (1 << 6)
#define PERIPH_RST3_UART3               (1 << 7)
#define PERIPH_RST3_UART4               (1 << 8)
#define PERIPH_RST3_SSP                 (1 << 9)
#define PERIPH_RST3_PWM                 (1 << 10)
#define PERIPH_RST3_BLPWM               (1 << 11)
#define PERIPH_RST3_TSENSOR             (1 << 12)
#define PERIPH_RST3_DAPB                (1 << 18)
#define PERIPH_RST3_HKADC               (1 << 19)
#define PERIPH_RST3_CODEC_SSI           (1 << 20)
#define PERIPH_RST3_PMUSSI1             (1 << 22)

#endif /* __HI6220_REGS_PERI_H__ */
