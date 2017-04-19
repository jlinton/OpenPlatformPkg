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

#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DevicePathLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>

#include <Library/IoLib.h>
#include "Hi6220RegsPeri.h"

VOID
EFIAPI
HiKeyInitPeripherals (
  IN VOID
  )
{
  UINT32     Data, Bits;

  /* take I2C0/I2C1/I2C2/SPI0 out of reset */
  Bits = PERIPH_RST3_I2C0 | PERIPH_RST3_I2C1 | PERIPH_RST3_I2C2 | \
	 PERIPH_RST3_SSP;
  MmioWrite32 (SC_PERIPH_RSTDIS3, Bits);

  do {
    Data = MmioRead32 (SC_PERIPH_RSTSTAT3);
  } while (Data & Bits);

  // make sure mmc2 is out of reset
  Bits = PERI_RST0_MMC2;
  MmioWrite32 (SC_PERIPH_RSTDIS0, Bits);
  do {
    Data = MmioRead32 (SC_PERIPH_RSTSTAT0);
  } while (Data & Bits);

  /* enable wifi/bt clock */
  MmioWrite32 (PMUSSI_REG_EX (SC_PMIC_CLK19M2_600_586_EN), 1); //enable 32khz clock
  // enable wifi clock
  Data  = MmioRead32 (PMUSSI_REG_EX (SC_PMIC_WLAN_CLK));
  Data |= SC_PMIC_WLAN_CLK_ENABLE;
  MmioWrite32 (PMUSSI_REG_EX (SC_PMIC_WLAN_CLK), Data);
}


EFI_STATUS
EFIAPI
HiKeyEntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS           Status;

  Status = EFI_SUCCESS; 
  HiKeyInitPeripherals ();

  return Status;
}
