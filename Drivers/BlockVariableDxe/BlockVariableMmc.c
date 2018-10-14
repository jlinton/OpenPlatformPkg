/** @file
  This file implements just a read/write method for the embedded emmc 
  On the hikey, the idea being that the main DwEmmcDxe is going to init
  everything, and we just want enough left over to flush the variables to 
  the mmc partition.

  Copyright (c) 2014-2017, Linaro Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/MmcHost.h>

#include "DwEmmc.h"

DWEMMC_IDMAC_DESCRIPTOR   *gpIdmacDesc;
extern EFI_PHYSICAL_ADDRESS gpIdmacDescBuf;
extern EFI_PHYSICAL_ADDRESS gpIdmacDescBufPa;
extern EFI_PHYSICAL_ADDRESS DwEmmcDxeBaseAddress;

EFI_STATUS
PrepareDmaData (
  IN DWEMMC_IDMAC_DESCRIPTOR*    IdmacDesc,
  IN EFI_PHYSICAL_ADDRESS       IdmacDescPa,
  IN UINTN                      Length,
  IN UINT32*                    Buffer
  )
{
  UINTN  Cnt, Blks, Idx, LastIdx;

  Cnt = (Length + DWEMMC_DMA_BUF_SIZE - 1) / DWEMMC_DMA_BUF_SIZE;
  Blks = (Length + DWEMMC_BLOCK_SIZE - 1) / DWEMMC_BLOCK_SIZE;
  Length = DWEMMC_BLOCK_SIZE * Blks;

  for (Idx = 0; Idx < Cnt; Idx++) {
    (IdmacDesc + Idx)->Des0 = DWEMMC_IDMAC_DES0_OWN | DWEMMC_IDMAC_DES0_CH |
                              DWEMMC_IDMAC_DES0_DIC;
    (IdmacDesc + Idx)->Des1 = DWEMMC_IDMAC_DES1_BS1(DWEMMC_DMA_BUF_SIZE);
    /* Buffer Address */
    (IdmacDesc + Idx)->Des2 = (UINT32)((UINTN)Buffer + DWEMMC_DMA_BUF_SIZE * Idx);
    /* Next Descriptor Address */
    (IdmacDesc + Idx)->Des3 = (UINT32)((UINTN)IdmacDesc +
                                       (sizeof(DWEMMC_IDMAC_DESCRIPTOR) * (Idx + 1)));
  }
  /* First Descriptor */
  IdmacDesc->Des0 |= DWEMMC_IDMAC_DES0_FS;
  /* Last Descriptor */
  LastIdx = Cnt - 1;
  (IdmacDesc + LastIdx)->Des0 |= DWEMMC_IDMAC_DES0_LD;
  (IdmacDesc + LastIdx)->Des0 &= ~(DWEMMC_IDMAC_DES0_DIC | DWEMMC_IDMAC_DES0_CH);
  (IdmacDesc + LastIdx)->Des1 = DWEMMC_IDMAC_DES1_BS1(Length -
                                                      (LastIdx * DWEMMC_DMA_BUF_SIZE));
  /* Set the Next field of Last Descriptor */
  (IdmacDesc + LastIdx)->Des3 = 0;
  MmioWrite32 (DWEMMC_DBADDR, (UINT32)(IdmacDescPa));

  return EFI_SUCCESS;
}

EFI_STATUS
SendCommand (
  IN MMC_CMD                    MmcCmd,
  IN UINT32                     Argument
  )
{
  UINT32      Data, ErrMask;

  int max_wait = 100000;

  // Wait until MMC is idle
  do {
    Data = MmioRead32 (DWEMMC_STATUS);
	max_wait--;
  } while ((Data & DWEMMC_STS_DATA_BUSY) && (max_wait));

  MmioWrite32 (DWEMMC_RINTSTS, ~0);
  MmioWrite32 (DWEMMC_CMDARG, Argument);
  MmioWrite32 (DWEMMC_CMD, MmcCmd);

  ErrMask = DWEMMC_INT_EBE | DWEMMC_INT_HLE | DWEMMC_INT_RTO |
            DWEMMC_INT_RCRC | DWEMMC_INT_RE;
  ErrMask |= DWEMMC_INT_DCRC | DWEMMC_INT_DRT | DWEMMC_INT_SBE;
  max_wait = 100;
  do {
    MicroSecondDelay(500);
    Data = MmioRead32 (DWEMMC_RINTSTS);

    if (Data & ErrMask) {
      return EFI_DEVICE_ERROR;
    }
    if (Data & DWEMMC_INT_DTO) {     // Transfer Done
      break;
    }
	max_wait--;
  } while ((!(Data & DWEMMC_INT_CMD_DONE)) && (max_wait));
  return EFI_SUCCESS;
}


VOID
StartDma (
  UINTN    Length
  )
{
  UINT32 Data;

  Data = MmioRead32 (DWEMMC_CTRL);
  Data |= DWEMMC_CTRL_INT_EN | DWEMMC_CTRL_DMA_EN | DWEMMC_CTRL_IDMAC_EN;
  MmioWrite32 (DWEMMC_CTRL, Data);
  Data = MmioRead32 (DWEMMC_BMOD);
  Data |= DWEMMC_IDMAC_ENABLE | DWEMMC_IDMAC_FB;
  MmioWrite32 (DWEMMC_BMOD, Data);

  MmioWrite32 (DWEMMC_BLKSIZ, DWEMMC_BLOCK_SIZE);
  MmioWrite32 (DWEMMC_BYTCNT, Length);
}


EFI_STATUS
DwEmmcReadBlockData (
  IN EFI_LBA                    Lba,
  IN UINTN                      Length,
  IN UINT32*                    Buffer,
  IN UINT32*                    BufferPa
  )
{
  EFI_STATUS  Status;
  UINT32      DescPages, CountPerPage, Count;
  UINT32      mDwEmmcCommand;
  //EFI_TPL     Tpl;

//  Tpl = gBS->RaiseTPL (TPL_NOTIFY);

  CountPerPage = EFI_PAGE_SIZE / 16; // (4k/16)
  Count = (Length + DWEMMC_DMA_BUF_SIZE - 1) / DWEMMC_DMA_BUF_SIZE; //(len+page)/4k,  transfer len in buf pages
  DescPages = (Count + CountPerPage - 1) / CountPerPage;

  InvalidateDataCacheRange (Buffer, Length);

  gpIdmacDesc = (DWEMMC_IDMAC_DESCRIPTOR *)gpIdmacDescBuf;
  Status = PrepareDmaData (gpIdmacDesc, gpIdmacDescBufPa, Length, BufferPa);
  if (EFI_ERROR (Status)) {
    goto out;
  }

  WriteBackDataCacheRange (gpIdmacDesc, DescPages * EFI_PAGE_SIZE);
  StartDma (Length);

  mDwEmmcCommand = MMC_INDX(17) | BIT_CMD_RESPONSE_EXPECT | BIT_CMD_CHECK_RESPONSE_CRC |
           BIT_CMD_DATA_EXPECTED | BIT_CMD_READ |
           BIT_CMD_WAIT_PRVDATA_COMPLETE | BIT_CMD_USE_HOLD_REG | BIT_CMD_START;

  Status = SendCommand (mDwEmmcCommand, Lba);

//  DEBUG ((DEBUG_ERROR, "read data2, mDwEmmcCommand:%x, mDwEmmcArgument:%x, Status:%r\n", mDwEmmcCommand, Lba, Status));
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to read data, mDwEmmcCommand:%x, mDwEmmcArgument:%x, Status:%r\n", mDwEmmcCommand, Lba, Status));
    goto out;
  }
out:
  // Restore Tpl
  //gBS->RestoreTPL (Tpl);
  return Status;
}

EFI_STATUS
DwEmmcWriteBlockData (
  IN EFI_LBA                    Lba,
  IN UINTN                      Length,
  IN UINT32*                    Buffer,
  IN UINT32*                    BufferPa
  )
{
  EFI_STATUS  Status;
  UINT32      DescPages, CountPerPage, Count;
  UINT32      mDwEmmcCommand;
//  EFI_TPL     Tpl;

//  Tpl = gBS->RaiseTPL (TPL_NOTIFY);
  

  CountPerPage = EFI_PAGE_SIZE / 16;
  Count = (Length + DWEMMC_DMA_BUF_SIZE - 1) / DWEMMC_DMA_BUF_SIZE;
  DescPages = (Count + CountPerPage - 1) / CountPerPage;

  WriteBackDataCacheRange (Buffer, Length);

  gpIdmacDesc = (DWEMMC_IDMAC_DESCRIPTOR *)gpIdmacDescBuf;
  Status = PrepareDmaData (gpIdmacDesc,gpIdmacDescBufPa, Length, BufferPa);
  if (EFI_ERROR (Status)) {
    goto out;
  }

  WriteBackDataCacheRange (gpIdmacDesc, DescPages * EFI_PAGE_SIZE);
  StartDma (Length);

  mDwEmmcCommand = MMC_INDX(24) | BIT_CMD_RESPONSE_EXPECT | 
	               BIT_CMD_CHECK_RESPONSE_CRC |
	               BIT_CMD_DATA_EXPECTED | BIT_CMD_WRITE |
                   BIT_CMD_WAIT_PRVDATA_COMPLETE | 
	               BIT_CMD_USE_HOLD_REG | BIT_CMD_START;

  Status = SendCommand (mDwEmmcCommand, Lba);
//  DEBUG ((DEBUG_ERROR, "write data2, mDwEmmcCommand:%x, mDwEmmcArgument:%x, Status:%r\n", mDwEmmcCommand, Lba, Status));
  if (EFI_ERROR (Status))  {
    DEBUG ((DEBUG_ERROR, "Failed to write data, mDwEmmcCommand:%x, mDwEmmcArgument:%x, Status:%r\n", mDwEmmcCommand, Lba, Status));
//    goto out;
  }
out:
  // Restore Tpl
  //gBS->RestoreTPL (Tpl);
  return Status;
}
