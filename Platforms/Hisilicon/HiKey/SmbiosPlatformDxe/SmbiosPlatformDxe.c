/** @file
  This driver installs SMBIOS information for the HiKey platform

  Copyright (c) 2015, ARM Limited. All rights reserved.
  Copyright (c) 2016, Hisilicon Ltd and Contributors. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <ArmPlatform.h>
#include <IndustryStandard/SmBios.h>
#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <PiDxe.h>
#include <Protocol/Smbios.h>

#include <Hi6220.h>

STATIC BOOLEAN mExtraMemoryPresent = FALSE;

#define HIKEY_EXTRA_SYSTEM_MEMORY_BASE  0x40000000
#define HIKEY_EXTRA_SYSTEM_MEMORY_SIZE  0x40000000

#define TYPE0_STRINGS                                    \
  "EFI Development Kit II / Hisilicon\0" /* Vendor */      \
  "EDK II\0"                             /* BiosVersion */ \
  __DATE__"\0"                           /* BiosReleaseDate */

#define TYPE1_STRINGS                                   \
  "LeMaker\0"                        /* Manufacturer */ \
  "96Boards LeMaker HiKey\0"         /* Product Name */ \
  "None\0"                           /* Version */      \
  "                    \0"           /* 20 character buffer */

#define TYPE2_STRINGS                                     \
  "LeMaker\0"                        /* Manufacturer */   \
  "96Boards LeMaker HiKey\0"         /* Product Name */   \
  "R0\0"                             /* Version */        \
  "Serial Not Set\0"                 /* Serial */         \
  "Base of Chassis\0"                /* board location */ \
  "R0\0"                             /* Version */

#define TYPE3_STRINGS                                   \
  "LeMaker\0"                        /* Manufacturer */ \
  "None\0"                           /* Version */      \
  "Serial Not Set\0"                 /* Serial  */

#define TYPE4_STRINGS                                             \
  "BGA-653\0"                        /* socket type */            \
  "Hisilicon\0"                      /* manufactuer */            \
  "Cortex-A53\0"                     /* processor description */  \
  "0xd03\0"                          /* A53 part number */

#define TYPE7_STRINGS                              \
  "L1 Instruction\0"                 /* L1I  */    \
  "L1 Data\0"                        /* L1D  */    \
  "L2\0"                             /* L2   */

#define TYPE8_STRINGS                              \
  "USB 0\0"                           /* Port0 */   \
  "USB 1\0"                           /* Port1 */   \
  "USB_OTG 0\0"                       /* Port2 */   \
  "HDMI 0\0"                          /* port3 */

#define TYPE9_STRINGS                              \
  "MMC 0\0"                           /* Slot0 */

#define TYPE16_STRINGS                             \
  "\0"                               /* nothing */

#define TYPE17_STRINGS                                       \
  "RIGHT SIDE\0"                     /* location */          \
  "BANK 0\0"                         /* bank description */  \
  "Samsung DDR\0"                    /* Manufacturer */      \
  "Elpida DDR\0"                     /* Manufacturer */      \
  "Hynix DDR\0"                      /* Manufacturer */

#define TYPE19_STRINGS                             \
  "\0"                               /* nothing */

#define TYPE32_STRINGS                             \
  "\0"                               /* nothing */

//
// Type definition and contents of the default SMBIOS table.
// This table covers only the minimum structures required by
// the SMBIOS specification (section 6.2, version 3.0)
//
#pragma pack(1)
typedef struct {
  SMBIOS_TABLE_TYPE0 Base;
  INT8              Strings[sizeof(TYPE0_STRINGS)];
} ARM_TYPE0;

typedef struct {
  SMBIOS_TABLE_TYPE1 Base;
  UINT8              Strings[sizeof(TYPE1_STRINGS)];
} ARM_TYPE1;

typedef struct {
  SMBIOS_TABLE_TYPE2 Base;
  UINT8              Strings[sizeof(TYPE2_STRINGS)];
} ARM_TYPE2;

typedef struct {
  SMBIOS_TABLE_TYPE3 Base;
  UINT8              Strings[sizeof(TYPE3_STRINGS)];
} ARM_TYPE3;

typedef struct {
  SMBIOS_TABLE_TYPE4 Base;
  UINT8              Strings[sizeof(TYPE4_STRINGS)];
} ARM_TYPE4;

typedef struct {
  SMBIOS_TABLE_TYPE7 Base;
  UINT8              Strings[sizeof(TYPE7_STRINGS)];
} ARM_TYPE7;

typedef struct {
  SMBIOS_TABLE_TYPE8 Base;
  UINT8              Strings[sizeof(TYPE8_STRINGS)];
} ARM_TYPE8;

typedef struct {
  SMBIOS_TABLE_TYPE9 Base;
  UINT8              Strings[sizeof(TYPE9_STRINGS)];
} ARM_TYPE9;

typedef struct {
  SMBIOS_TABLE_TYPE16 Base;
  UINT8              Strings[sizeof(TYPE16_STRINGS)];
} ARM_TYPE16;

typedef struct {
  SMBIOS_TABLE_TYPE17 Base;
  UINT8              Strings[sizeof(TYPE17_STRINGS)];
} ARM_TYPE17;

typedef struct {
  SMBIOS_TABLE_TYPE19 Base;
  UINT8              Strings[sizeof(TYPE19_STRINGS)];
} ARM_TYPE19;

typedef struct {
  SMBIOS_TABLE_TYPE32 Base;
  UINT8              Strings[sizeof(TYPE32_STRINGS)];
} ARM_TYPE32;

// SMBIOS tables often refrence each other using
// fixed constants, define a list of these contants
// for our hardcoded tables
enum SMBIOS_REFRENCE_HANDLES {
  SMBIOS_HANDLE_A57_L1I = 0x1000,
  SMBIOS_HANDLE_A57_L1D,
  SMBIOS_HANDLE_A57_L2,
  SMBIOS_HANDLE_A53_L1I,
  SMBIOS_HANDLE_A53_L1D,
  SMBIOS_HANDLE_A53_L2,
  SMBIOS_HANDLE_MOTHERBOARD,
  SMBIOS_HANDLE_CHASSIS,
  SMBIOS_HANDLE_A72_CLUSTER,
  SMBIOS_HANDLE_A57_CLUSTER,
  SMBIOS_HANDLE_A53_CLUSTER,
  SMBIOS_HANDLE_MEMORY,
  SMBIOS_HANDLE_DIMM
};

#define SERIAL_LEN 10  //this must be less than the buffer len allocated in the type1 structure

#pragma pack()

// BIOS information (section 7.1)
STATIC ARM_TYPE0 mArmDefaultType0 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_BIOS_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE0),      // UINT8 Length
      SMBIOS_HANDLE_PI_RESERVED,
    },
    1,     // SMBIOS_TABLE_STRING       Vendor
    2,     // SMBIOS_TABLE_STRING       BiosVersion
    0xE800,// UINT16                    BiosSegment
    3,     // SMBIOS_TABLE_STRING       BiosReleaseDate
    0,     // UINT8                     BiosSize
    {
      0,0,0,0,0,0,
      0,
      0,
      0,
      0,
      1, //BIOS upgradable
      0, 0, 0,
      0,
      1, //selectable boot
    },  // MISC_BIOS_CHARACTERISTICS BiosCharacteristics
    {      // BIOSCharacteristicsExtensionBytes[2]
      0x2,
      0xC,
    },
    0,     // UINT8                     SystemBiosMajorRelease
    0,     // UINT8                     SystemBiosMinorRelease
    0xFF,  // UINT8                     EmbeddedControllerFirmwareMajorRelease
    0xFF   // UINT8                     EmbeddedControllerFirmwareMinorRelease
  },
  // Text strings (unformatted area)
  TYPE0_STRINGS
};

// System information (section 7.2)
STATIC CONST ARM_TYPE1 mArmDefaultType1 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_SYSTEM_INFORMATION,
      sizeof(SMBIOS_TABLE_TYPE1),
      SMBIOS_HANDLE_PI_RESERVED,
    },
    1,     //Manufacturer
    2,     //Product Name
    3,     //Version
    4,     //Serial
    { 0x8a95d198, 0x7f46, 0x11e5, { 0xbf,0x8b,0x08,0x00,0x27,0x04,0xd4,0x8e }},    //UUID
    6,     //Wakeup type
    0,     //SKU
    0,     //Family
  },
  // Text strings (unformatted)
  TYPE1_STRINGS
};

// Baseboard (section 7.3)
STATIC ARM_TYPE2 mArmDefaultType2 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_BASEBOARD_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE2),           // UINT8 Length
      SMBIOS_HANDLE_MOTHERBOARD,
    },
    1,    //Manufacturer
    2,    //Product Name
    3,    //Version
    4,    //Serial
    0,    //Asset tag
    {1},  //motherboard, not replaceable
    5,    //location of board
    SMBIOS_HANDLE_CHASSIS,
    BaseBoardTypeMotherBoard,
    1,
    {SMBIOS_HANDLE_A53_CLUSTER}, //,SMBIOS_HANDLE_A53_CLUSTER,SMBIOS_HANDLE_MEMORY},
  },
  TYPE2_STRINGS
};

// Enclosure
STATIC CONST ARM_TYPE3 mArmDefaultType3 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_SYSTEM_ENCLOSURE, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE3),      // UINT8 Length
      SMBIOS_HANDLE_CHASSIS,
    },
    1,   //Manufacturer
    4,   //enclosure type (low profile desktop)
    2,   //version
    3,   //serial
    0,   //asset tag
    ChassisStateUnknown,   //boot chassis state
    ChassisStateSafe,      //power supply state
    ChassisStateSafe,      //thermal state
    ChassisSecurityStatusNone,   //security state
    {0,0,0,0,}, //OEM defined
    1,  //1U height
    1,  //number of power cords
    0,  //no contained elements
  },
  TYPE3_STRINGS
};

// Processor
STATIC CONST ARM_TYPE4 mArmDefaultType4_a53 = {
  {
    {   // SMBIOS_STRUCTURE Hdr
        EFI_SMBIOS_TYPE_PROCESSOR_INFORMATION, // UINT8 Type
        sizeof (SMBIOS_TABLE_TYPE4),           // UINT8 Length
        SMBIOS_HANDLE_A53_CLUSTER,
    },
    1, //socket type
    3, //processor type CPU
    ProcessorFamilyIndicatorFamily2, //processor family, acquire from field2
    2, //manufactuer
    {{0,},{0.}}, //processor id
    4, //version
    {0,0,0,0,0,1}, //voltage
    0, //external clock
    1200, //max speed
    1200, //current speed
    0x41, //status
    ProcessorUpgradeOther,
    SMBIOS_HANDLE_A53_L1I, //l1 cache handle
    SMBIOS_HANDLE_A53_L2, //l2 cache handle
    0xFFFF, //l3 cache handle
    0, //serial not set
    0, //asset not set
    4, //part number
    8, //core count in socket
    8, //enabled core count in socket
    8, //threads per socket
    0xEC, // processor characteristics
    ProcessorFamilyARM, //ARM core
  },
  TYPE4_STRINGS
};

// Cache
STATIC CONST ARM_TYPE7 mArmDefaultType7_a53_l1i = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_CACHE_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE7),       // UINT8 Length
      SMBIOS_HANDLE_A53_L1I,
    },
    1,
    0x380, //L1 enabled, unknown WB
    32, //32k i cache max
    32, //32k installed
    {0,1}, //SRAM type
    {0,1}, //SRAM type
    0, //unkown speed
    CacheErrorParity, //parity checking
    CacheTypeInstruction, //instruction cache
    CacheAssociativity2Way, //two way
  },
  TYPE7_STRINGS
};

STATIC CONST ARM_TYPE7 mArmDefaultType7_a53_l1d = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_CACHE_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE7),       // UINT8 Length
      SMBIOS_HANDLE_A53_L1D,
    },
    2,
    0x180, //L1 enabled, WB
    32, //32k d cache max
    32, //32k installed
    {0,1}, //SRAM type
    {0,1}, //SRAM type
    0, //unkown speed
    CacheErrorSingleBit, //ECC checking
    CacheTypeData, //instruction cache
    CacheAssociativity4Way, //four way associative
  },
  TYPE7_STRINGS
};

STATIC CONST ARM_TYPE7 mArmDefaultType7_a53_l2 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_CACHE_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE7),       // UINT8 Length
      SMBIOS_HANDLE_A53_L2,
    },
    3,
    0x181, //L2 enabled, WB
    512, //512K D cache max
    512, //512K installed
    {0,1}, //SRAM type
    {0,1}, //SRAM type
    0, //unkown speed
    CacheErrorSingleBit, //ECC checking
    CacheTypeUnified, //instruction cache
    CacheAssociativity16Way, //16 way associative
  },
  TYPE7_STRINGS
};

// Ports
STATIC CONST ARM_TYPE8 mArmDefaultType8_usb0 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_PORT_CONNECTOR_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE8),                 // UINT8 Length
      SMBIOS_HANDLE_PI_RESERVED,
    },
    0,
    PortConnectorTypeNone,
    1, //port 0
    PortConnectorTypeUsb,
    PortTypeUsb,
  },
  TYPE8_STRINGS
};

STATIC CONST ARM_TYPE8 mArmDefaultType8_usb1 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_PORT_CONNECTOR_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE8),                 // UINT8 Length
      SMBIOS_HANDLE_PI_RESERVED,
    },
    0,
    PortConnectorTypeNone,
    2, //port 1
    PortConnectorTypeUsb,
    PortTypeUsb,
  },
  TYPE8_STRINGS
};

STATIC CONST ARM_TYPE8 mArmDefaultType8_usb_otg0 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_PORT_CONNECTOR_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE8),                 // UINT8 Length
      SMBIOS_HANDLE_PI_RESERVED,
    },
    0,
    PortConnectorTypeNone,
    3, //port 2
    PortConnectorTypeUsb,
    PortTypeUsb,
  },
  TYPE8_STRINGS
};

STATIC CONST ARM_TYPE8 mArmDefaultType8_hdmi0 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_PORT_CONNECTOR_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE8),                 // UINT8 Length
      SMBIOS_HANDLE_PI_RESERVED,
    },
    0,
    PortConnectorTypeNone,
    4, //port 3
    PortConnectorTypeNone,
    PortTypeOther, //hdmi
  },
  TYPE8_STRINGS
};

// Slots
STATIC CONST ARM_TYPE9 mArmDefaultType9 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_SYSTEM_SLOTS, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE9),  // UINT8 Length
      SMBIOS_HANDLE_PI_RESERVED,
    },
    1, //slot 0
    SlotTypeOther,
    SlotDataBusWidthOther,
    SlotUsageAvailable,
    SlotLengthOther,
    0,
    {1}, //unknown
    {1,0,1},  //PME and SMBUS
    0,
    0,
    0,
  },
  TYPE9_STRINGS
};

// Memory array
STATIC CONST ARM_TYPE16 mArmDefaultType16 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_PHYSICAL_MEMORY_ARRAY, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE16),          // UINT8 Length
      SMBIOS_HANDLE_MEMORY,
    },
    MemoryArrayLocationSystemBoard, //on motherboard
    MemoryArrayUseSystemMemory,     //system RAM
    MemoryErrorCorrectionNone,      //HiKey doesn't have ECC RAM
    0x100000, //1GB
    0xFFFE,   //No error information structure
    0x1,      //soldered memory
  },
  TYPE16_STRINGS
};

// Memory device
STATIC CONST ARM_TYPE17 mArmDefaultType17 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_MEMORY_DEVICE, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE17),  // UINT8 Length
      SMBIOS_HANDLE_DIMM,
    },
    SMBIOS_HANDLE_MEMORY, //array to which this module belongs
    0xFFFE,               //no errors
    32,     //single chip, no ECC is 32bits (for ecc this would be 72)
    32,     //data width of this device (32-bits)
    0,      //DDR size
    0x05,   //single chip
    0,      //not part of a set
    1,      //right side of board
    2,      //bank 0
//  MemoryTypeLpddr3, //LP DDR3, isn't defined yet
    MemoryTypeDdr3,                  //LP DDR3
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}, //unbuffered
    0,                               //DDR speed
    0, //varies between diffrent production runs
    0, //serial
    0, //asset tag
    0, //part number
    1, //rank
  },
  TYPE17_STRINGS
};

// Memory array mapped addr, this structure overriden by InstallMemoryStructure
STATIC CONST ARM_TYPE19 mArmDefaultType19 = {
  {
    {  // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_MEMORY_ARRAY_MAPPED_ADDRESS, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE19),                // UINT8 Length
      SMBIOS_HANDLE_PI_RESERVED,
    },
    0xFFFFFFFF, //invalid, look at extended addr field
    0xFFFFFFFF,
    SMBIOS_HANDLE_DIMM, //handle
    1,
    0x080000000,        //starting addr of first 2GB
    0x100000000,        //ending addr of first 2GB
  },
  TYPE19_STRINGS
};

// System boot info
STATIC CONST ARM_TYPE32 mArmDefaultType32 = {
  {
    { // SMBIOS_STRUCTURE Hdr
      EFI_SMBIOS_TYPE_SYSTEM_BOOT_INFORMATION, // UINT8 Type
      sizeof (SMBIOS_TABLE_TYPE32),            // UINT8 Length
      SMBIOS_HANDLE_PI_RESERVED,
    },
    {0,0,0,0,0,0},                             //reserved
    BootInformationStatusNoError,
  },
  TYPE32_STRINGS
};

STATIC CONST VOID *DefaultCommonTables[]=
{
  &mArmDefaultType0,
  &mArmDefaultType1,
  &mArmDefaultType2,
  &mArmDefaultType3,
  &mArmDefaultType4_a53,
  &mArmDefaultType7_a53_l1i,
  &mArmDefaultType7_a53_l1d,
  &mArmDefaultType7_a53_l2,
  &mArmDefaultType8_usb0,
  &mArmDefaultType8_usb1,
  &mArmDefaultType8_usb_otg0,
  &mArmDefaultType8_hdmi0,
  &mArmDefaultType9,
  &mArmDefaultType16,
  &mArmDefaultType32,
  NULL
};

/**
   Installs a memory device (type17)

   @param  Smbios               SMBIOS protocol

**/
EFI_STATUS
InstallMemoryDeviceStructure (
  IN EFI_SMBIOS_PROTOCOL       *Smbios
  )
{
  UINT32                      Data;
  UINT32                      VendorID;
  UINT64                      MemorySize;
  UINT32                      SpeedProfile;
  UINT64                      RegionMapCount;
  EFI_SMBIOS_HANDLE           SmbiosHandle;
  ARM_TYPE17                  MemoryDevice;
  EFI_STATUS                  Status = EFI_SUCCESS;

  CopyMem (&MemoryDevice, &mArmDefaultType17, sizeof (ARM_TYPE17));

  // LPDDR manufacturer
  MmioWrite32 ((MDDRC_DMC_BASE + 0x210), 0x57);
  MmioWrite32 ((MDDRC_DMC_BASE + 0x218), 0x10000);
  MmioWrite32 ((MDDRC_DMC_BASE + 0x00c), 0x1);

  do {
    Data = MmioRead32 (MDDRC_DMC_BASE + 0x00C);
  } while (Data & 1);

  Data = MmioRead32 (MDDRC_DMC_BASE + 0x4A8);
  VendorID = Data & 0xFF;

  switch (VendorID) {
    case 1:   // Samsung DDR
      MemoryDevice.Base.Manufacturer = 3;
      break;
    case 3:   // Elpida DDR
      MemoryDevice.Base.Manufacturer = 4;
      break;
    case 6:   // Hynix DDR
      MemoryDevice.Base.Manufacturer = 5;
      break;
    default:  // Unknown DDR
      MemoryDevice.Base.Manufacturer = 0;
      break;
  }

  // LPDDR size
  RegionMapCount = 0;
  while (MmioRead32 (MDDRC_AXI_BASE + AXI_REGION_MAP_OFFSET (RegionMapCount)) != 0) {
    RegionMapCount++;
  }
  Data = MmioRead32 (MDDRC_AXI_BASE + AXI_REGION_MAP_OFFSET (RegionMapCount - 1));
  MemorySize = 16 << ((Data >> 8) & 0x7);
  MemorySize += Data << 24;
  MemoryDevice.Base.Size = (UINT16)((MemorySize << 20) / SIZE_1MB);

  if (MemoryDevice.Base.Size > SIZE_1GB) {
    mExtraMemoryPresent = TRUE;
  }

  // LPDDR speed
  SpeedProfile = MmioRead32 (PMCTRL_DDRCLKDIVCFG);
  switch (SpeedProfile) {
    case 0x11111:  // 533 Mhz profile
      MemoryDevice.Base.Speed = 533;
      break;
    case 0x1003:   // 800 MHz profile
      MemoryDevice.Base.Speed = 800;
      break;
    default:  // Unknown profile
      break;
  }

  SmbiosHandle = MemoryDevice.Base.Hdr.Handle;

  Status = Smbios->Add (
    Smbios,
    NULL,
    &SmbiosHandle,
    (EFI_SMBIOS_TABLE_HEADER*) &MemoryDevice
  );

  return Status;
}

/**
   Installs a memory descriptor (type19) for the given address range

   @param  Smbios               SMBIOS protocol

**/
EFI_STATUS
InstallMemoryStructure (
  IN EFI_SMBIOS_PROTOCOL       *Smbios,
  IN UINT64                    StartingAddress,
  IN UINT64                    RegionLength
  )
{
  EFI_SMBIOS_HANDLE         SmbiosHandle;
  ARM_TYPE19                MemoryDescriptor;
  EFI_STATUS                Status = EFI_SUCCESS;

  CopyMem (&MemoryDescriptor, &mArmDefaultType19, sizeof (ARM_TYPE19));

  MemoryDescriptor.Base.ExtendedStartingAddress = StartingAddress;
  MemoryDescriptor.Base.ExtendedEndingAddress = StartingAddress + RegionLength;
  SmbiosHandle = MemoryDescriptor.Base.Hdr.Handle;

  Status = Smbios->Add (
    Smbios,
    NULL,
    &SmbiosHandle,
    (EFI_SMBIOS_TABLE_HEADER*) &MemoryDescriptor
  );

  return Status;
}

/**
   Install a whole table worth of structructures

   @parm
**/
EFI_STATUS
InstallStructures (
   IN EFI_SMBIOS_PROTOCOL       *Smbios,
   IN CONST VOID *DefaultTables[]
   )
{
    EFI_STATUS                Status = EFI_SUCCESS;
    EFI_SMBIOS_HANDLE         SmbiosHandle;

    int TableEntry;
    for ( TableEntry=0; DefaultTables[TableEntry] != NULL; TableEntry++ )
    {
      SmbiosHandle = ((EFI_SMBIOS_TABLE_HEADER*)DefaultTables[TableEntry])->Handle;
      Status = Smbios->Add (
        Smbios,
        NULL,
        &SmbiosHandle,
        (EFI_SMBIOS_TABLE_HEADER*) DefaultTables[TableEntry]
	    );
      if (EFI_ERROR(Status))
        break;
    }
    return Status;
}

/**
   Install all structures from the DefaultTables structure

   @param  Smbios               SMBIOS protocol

**/
EFI_STATUS
InstallAllStructures (
   IN EFI_SMBIOS_PROTOCOL       *Smbios
   )
{
  EFI_STATUS                Status = EFI_SUCCESS;

  // Fixup some table values
  mArmDefaultType0.Base.SystemBiosMajorRelease = (PcdGet32 (PcdFirmwareRevision) >> 16) & 0xFF;
  mArmDefaultType0.Base.SystemBiosMinorRelease = PcdGet32 (PcdFirmwareRevision) & 0xFF;
  mArmDefaultType2.Base.Version = 0;

  //
  // Add all table entries
  //
  Status = InstallStructures (Smbios, DefaultCommonTables);
  ASSERT_EFI_ERROR (Status);

  // Generate memory device descriptios for vendor manufactuer, size, and speed
  Status = InstallMemoryDeviceStructure (Smbios);
  ASSERT_EFI_ERROR (Status);

  // Generate memory descriptors for the memory ranges we know about
  Status = InstallMemoryStructure (Smbios, PcdGet64 (PcdSystemMemoryBase), PcdGet64 (PcdSystemMemorySize));
  ASSERT_EFI_ERROR (Status);
  if (mExtraMemoryPresent) {
    Status = InstallMemoryStructure (Smbios, HIKEY_EXTRA_SYSTEM_MEMORY_BASE, HIKEY_EXTRA_SYSTEM_MEMORY_SIZE);
    ASSERT_EFI_ERROR (Status);
  }

  return Status;
}

/**
   Installs SMBIOS information for ARM platforms

   @param ImageHandle     Module's image handle
   @param SystemTable     Pointer of EFI_SYSTEM_TABLE

   @retval EFI_SUCCESS    Smbios data successfully installed
   @retval Other          Smbios data was not installed

**/
EFI_STATUS
EFIAPI
SmbiosTablePublishEntry (
  IN EFI_HANDLE           ImageHandle,
  IN EFI_SYSTEM_TABLE     *SystemTable
  )
{
  EFI_STATUS                Status;
  EFI_SMBIOS_PROTOCOL       *Smbios;

  //
  // Find the SMBIOS protocol
  //
  Status = gBS->LocateProtocol (
    &gEfiSmbiosProtocolGuid,
    NULL,
    (VOID**)&Smbios
  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = InstallAllStructures (Smbios);

  return Status;
}
