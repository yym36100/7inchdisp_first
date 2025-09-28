#include "deepseek.h"

const char* get_manufacturer_name(uint8_t mid) {
    switch(mid) {
        case 0x01: return "Panasonic";
        case 0x02: return "Toshiba";
        case 0x03: return "SanDisk";
        case 0x08: return "Silicon Power";
        case 0x18: return "Infineon";
        case 0x1b: return "Samsung";
        case 0x1d: return "ADATA";
        case 0x27: return "Phison";
        case 0x28: return "Lexar";
        case 0x31: return "Silicon Motion";
        case 0x41: return "Kingston";
        case 0x74: return "Transcend";
        case 0x76: return "Patriot";
        case 0x82: return "Sony";
        case 0x9c: return "Angelbird";
        case 0xc8: return "Toshiba (Generic)";
        default:   return "Unknown";
    }
}

const char* get_csd_structure_name(uint8_t csd_structure) {
    switch(csd_structure) {
        case 0: return "SDSC (Standard Capacity, <= 2GB)";
        case 1: return "SDHC (High Capacity, 2GB-32GB)";
        case 2: return "SDXC (Extended Capacity, 32GB-2TB)";
        default: return "Reserved";
    }
}

const char* get_speed_class_name(uint8_t tran_speed) {
    uint8_t time_value = (tran_speed >> 3) & 0x0F;
    uint8_t time_unit = tran_speed & 0x07;

    float freq_mhz = 0;
    switch (time_unit) {
        case 0: freq_mhz = 0.1; break;
        case 1: freq_mhz = 1.0; break;
        case 2: freq_mhz = 10.0; break;
        case 3: freq_mhz = 100.0; break;
        default: freq_mhz = 1.0;
    }

    float max_freq = freq_mhz * time_value;

    if (max_freq >= 50) return "UHS-I / Class 10";
    if (max_freq >= 25) return "Class 10";
    if (max_freq >= 12.5) return "Class 6";
    if (max_freq >= 10) return "Class 4";
    return "Class 2 or below";
}

void calculate_capacity_string(uint32_t device_size, uint8_t device_size_mul,
                              uint8_t read_bl_len, uint8_t csd_structure,
                              char* buffer, size_t buffer_size) {
    uint64_t capacity_bytes;
    uint32_t block_count;
    uint32_t block_size;

    if (csd_structure == 0) {
        // SDSC (Standard Capacity)
        block_size = 1 << read_bl_len;
        uint32_t multiplier = 1 << (device_size_mul + 2);
        block_count = (device_size + 1) * multiplier;
        capacity_bytes = (uint64_t)block_count * block_size;
    } else {
        // SDHC/SDXC (High/Extended Capacity)
        block_count = (device_size + 1) * 1024;
        block_size = 512;
        capacity_bytes = (uint64_t)block_count * block_size;
    }

    // Format capacity string
    if (capacity_bytes >= (1024ULL * 1024 * 1024 * 1024)) {
        snprintf(buffer, buffer_size, "%.1f TB (%llu bytes, %lu blocks)",
                capacity_bytes / (1024.0 * 1024 * 1024 * 1024),
                capacity_bytes, block_count);
    } else if (capacity_bytes >= (1024ULL * 1024 * 1024)) {
        snprintf(buffer, buffer_size, "%.1f GB (%llu bytes, %lu blocks)",
                capacity_bytes / (1024.0 * 1024 * 1024),
                capacity_bytes, block_count);
    } else if (capacity_bytes >= (1024ULL * 1024)) {
        snprintf(buffer, buffer_size, "%.1f MB (%llu bytes, %lu blocks)",
                capacity_bytes / (1024.0 * 1024),
                capacity_bytes, block_count);
    } else {
        snprintf(buffer, buffer_size, "%.1f KB (%llu bytes, %lu blocks)",
                capacity_bytes / 1024.0,
                capacity_bytes, block_count);
    }
}

void print_sd_card_info(HAL_SD_CardCSDTypeDef *csd, HAL_SD_CardCIDTypeDef *cid) {
    printf("\r\n");
    printf("+----------------------------------------+\r\n");
    printf("�           SD CARD INFORMATION          �\r\n");
    printf("+----------------------------------------+\r\n");
    printf("\r\n");

    // === CID INFORMATION ===
    printf("� CARD IDENTIFICATION (CID) REGISTER\r\n");
    printf("---------------------------------------------------\r\n");
    
    // Manufacturer and Basic Info
    printf("  Manufacturer:   0x%02X - %s\r\n",
           cid->ManufacturerID, get_manufacturer_name(cid->ManufacturerID));
    
    printf("  OEM ID:         0x%04X ('%c%c')\r\n",
           cid->OEM_AppliID,
           (cid->OEM_AppliID >> 8) & 0xFF, cid->OEM_AppliID & 0xFF);
    
    // Product Name (handling the split fields)
    char prod_name[6];
    snprintf(prod_name, sizeof(prod_name), "%c%c%c%c%c",
            (char)((cid->ProdName1 >> 24) & 0xFF),
            (char)((cid->ProdName1 >> 16) & 0xFF),
            (char)((cid->ProdName1 >> 8) & 0xFF),
            (char)(cid->ProdName1 & 0xFF),
            (char)cid->ProdName2);
    printf("  Product Name:   '%s' (0x%08lX%02X)\r\n",
           prod_name, cid->ProdName1, cid->ProdName2);
    
    printf("  Revision:       %d.%d\r\n",
           (cid->ProdRev >> 4) & 0x0F, cid->ProdRev & 0x0F);
    
    printf("  Serial Number:  %lu (0x%08lX)\r\n",
           cid->ProdSN, cid->ProdSN);
    
    // Manufacturing Date
    uint8_t manufact_year = (cid->ManufactDate >> 4) & 0xFF;
    uint8_t manufact_month = cid->ManufactDate & 0x0F;
    const char* months[] = {"", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    printf("  Manufactured:   %s %d (Week %d, 0x%03X)\r\n",
           manufact_month <= 12 ? months[manufact_month] : "???",
           2000 + manufact_year, manufact_year, cid->ManufactDate);

    printf("  CID CRC:        0x%02X\r\n", cid->CID_CRC);
    printf("\r\n");
    
    // === CSD INFORMATION ===
    printf("� CARD SPECIFIC DATA (CSD) REGISTER\r\n");
    printf("---------------------------------------------------\r\n");
    
    // Card Type and Capacity
    printf("  Card Type:      %s (CSD Structure: %d)\r\n",
           get_csd_structure_name(csd->CSDStruct), csd->CSDStruct);
    
    char capacity_str[128];
    calculate_capacity_string(csd->DeviceSize, csd->DeviceSizeMul,
                             csd->RdBlockLen, csd->CSDStruct,
                             capacity_str, sizeof(capacity_str));
    printf("  Capacity:       %s\r\n", capacity_str);
    
    // Speed Information
    uint8_t time_value = (csd->MaxBusClkFrec >> 3) & 0x0F;
    uint8_t time_unit = csd->MaxBusClkFrec & 0x07;
    const char* units[] = {"100 kHz", "1 MHz", "10 MHz", "100 MHz"};
    float max_freq = time_value * (time_unit == 0 ? 0.1 :
                                  time_unit == 1 ? 1.0 :
                                  time_unit == 2 ? 10.0 : 100.0);
    
    printf("  Max Frequency:  %.1f MHz (%s * %d)\r\n",
           max_freq,
           time_unit < 4 ? units[time_unit] : "?",
           time_value);
    printf("  Speed Class:    %s\r\n", get_speed_class_name(csd->MaxBusClkFrec));
    
    // Block and Transfer Parameters
    printf("  Block Size:     %d bytes (2^%d)\r\n",
           1 << csd->RdBlockLen, csd->RdBlockLen);
    
    printf("  Command Classes:0x%03X\r\n", csd->CardComdClasses);
    
    // Feature Support
    printf("  Features:\r\n");
    printf("    - Partial Block Read:    %s\r\n", csd->PartBlockRead ? "Yes" : "No");
    printf("    - Write Block Misalign:  %s\r\n", csd->WrBlockMisalign ? "Yes" : "No");
    printf("    - Read Block Misalign:   %s\r\n", csd->RdBlockMisalign ? "Yes" : "No");
    printf("    - DSR Implemented:       %s\r\n", csd->DSRImpl ? "Yes" : "No");
    
    if (csd->CSDStruct == 0) {
        // SDSC specific features
        printf("    - Erase Group Size:      %d blocks\r\n", csd->EraseGrSize + 1);
        printf("    - Erase Group Multiplier:%d\r\n", csd->EraseGrMul + 1);
        printf("    - Write Protect Groups:  %s, Size: %d sectors\r\n",
               csd->WrProtectGrEnable ? "Enabled" : "Disabled",
               csd->WrProtectGrSize + 1);
    }
    
    printf("  Write Speed Factor: %d (Write is ~%dx slower than read)\r\n",
           csd->WrSpeedFact, 1 << csd->WrSpeedFact);
    
    printf("  CSD CRC:        0x%02X\r\n", csd->CSD_CRC);
    printf("\r\n");
    
    // === SUMMARY ===
    printf("� SUMMARY\r\n");
    printf("---------------------------------------------------\r\n");
    printf("  This is a %s SD card from %s,\r\n",
           get_csd_structure_name(csd->CSDStruct),
           get_manufacturer_name(cid->ManufacturerID));
    printf("  manufactured around %s %d.\r\n",
           manufact_month <= 12 ? months[manufact_month] : "???",
           2000 + manufact_year);
    printf("  It supports %s speeds and has a serial number of %lu.\r\n",
           get_speed_class_name(csd->MaxBusClkFrec), cid->ProdSN);
    printf("\r\n");
}

#include "stm32h7xx_hal.h"
#include <stdio.h>

void print_sd_card_status(HAL_SD_CardStatusTypeDef *status) {
    printf("\r\n");
    printf("+----------------------------------------+\r\n");
    printf("¦           SD CARD STATUS               ¦\r\n");
    printf("+----------------------------------------+\r\n");
    printf("\r\n");

    // === BUS CONFIGURATION ===
    printf("¦ BUS CONFIGURATION\r\n");
    printf("---------------------------------------------------\r\n");
    printf("  Data Bus Width:        ");
    switch(status->DataBusWidth) {
        case 0: printf("1-bit mode\r\n"); break;
        case 1: printf("Reserved\r\n"); break;
        case 2: printf("4-bit mode\r\n"); break;
        default: printf("Unknown (%d)\r\n", status->DataBusWidth);
    }

    printf("  Secured Mode:          %s\r\n",
           status->SecuredMode ? "ACTIVE (Card locked)" : "Inactive");

    printf("  Card Type:             0x%04X", status->CardType);
    if(status->CardType & 0x1) printf(" [SDSC]");
    if(status->CardType & 0x2) printf(" [SDHC]");
    if(status->CardType & 0x4) printf(" [SDXC]");
    if(status->CardType & 0x8) printf(" [SDUC]");
    if(status->CardType & 0x100) printf(" [IO Card]");
    printf("\r\n");

    // === MEMORY CONFIGURATION ===
    printf("\r\n¦ MEMORY CONFIGURATION\r\n");
    printf("---------------------------------------------------\r\n");
    printf("  Protected Area Size:   %lu sectors", status->ProtectedAreaSize);
    if(status->ProtectedAreaSize > 0) {
        printf(" (~%.1f MB)", (status->ProtectedAreaSize * 512.0) / (1024*1024));
    }
    printf("\r\n");

    printf("  Allocation Unit Size:  %d sectors (%d KB)\r\n",
           status->AllocationUnitSize,
           status->AllocationUnitSize * 512 / 1024);

    printf("  UHS Allocation Unit:   %d sectors\r\n",
           status->UhsAllocationUnitSize);

    // === ERASE CONFIGURATION ===
    printf("\r\n¦ ERASE CONFIGURATION\r\n");
    printf("---------------------------------------------------\r\n");
    printf("  Erase Size:            %d sectors (%d KB)\r\n",
           status->EraseSize, status->EraseSize * 512 / 1024);

    printf("  Erase Timeout:         %d ms\r\n", status->EraseTimeout);
    printf("  Erase Offset:          %d sectors\r\n", status->EraseOffset);

    // === PERFORMANCE CHARACTERISTICS ===
    printf("\r\n¦ PERFORMANCE CHARACTERISTICS\r\n");
    printf("---------------------------------------------------\r\n");

    // Speed Class (Traditional)
    printf("  Speed Class:           ");
    if(status->SpeedClass == 0) printf("Class 0 (Default)\r\n");
    else if(status->SpeedClass <= 2) printf("Class %d (>= 2 MB/s)\r\n", status->SpeedClass);
    else if(status->SpeedClass <= 4) printf("Class %d (>= 4 MB/s)\r\n", status->SpeedClass);
    else if(status->SpeedClass <= 6) printf("Class %d (>= 6 MB/s)\r\n", status->SpeedClass);
    else if(status->SpeedClass <= 10) printf("Class %d (>= 10 MB/s)\r\n", status->SpeedClass);
    else printf("Unknown (%d)\r\n", status->SpeedClass);

    // UHS Speed Grade
    printf("  UHS Speed Grade:       ");
    switch(status->UhsSpeedGrade) {
        case 0: printf("Not UHS\r\n"); break;
        case 1: printf("UHS Speed Grade 1 (10 MB/s)\r\n"); break;
        case 3: printf("UHS Speed Grade 3 (30 MB/s)\r\n"); break;
        case 4: printf("UHS Speed Grade 4 (40 MB/s)\r\n"); break;
        case 6: printf("UHS Speed Grade 6 (60 MB/s)\r\n"); break;
        case 8: printf("UHS Speed Grade 8 (80 MB/s)\r\n"); break;
        case 10: printf("UHS Speed Grade 10 (100 MB/s)\r\n"); break;
        case 13: printf("UHS Speed Grade 13 (130 MB/s)\r\n"); break;
        default: printf("Unknown (%d)\r\n", status->UhsSpeedGrade);
    }

    // Video Speed Class
    printf("  Video Speed Class:     ");
    switch(status->VideoSpeedClass) {
        case 0: printf("Not VSC\r\n"); break;
        case 6: printf("V6 (6 MB/s)\r\n"); break;
        case 10: printf("V10 (10 MB/s)\r\n"); break;
        case 30: printf("V30 (30 MB/s)\r\n"); break;
        case 60: printf("V60 (60 MB/s)\r\n"); break;
        case 90: printf("V90 (90 MB/s)\r\n"); break;
        default: printf("Unknown (%d)\r\n", status->VideoSpeedClass);
    }

    // Performance Move
    printf("  Performance Move:      %d\r\n", status->PerformanceMove);

    // === SECURITY STATUS ===
    printf("\r\n¦ SECURITY STATUS\r\n");
    printf("---------------------------------------------------\r\n");

    if(status->SecuredMode) {
        printf("  ??  CARD IS LOCKED/SECURED\r\n");
        printf("  - Requires password for access\r\n");
        printf("  - Write operations may be restricted\r\n");
    } else {
        printf("  ? Card is unlocked and accessible\r\n");
    }

    if(status->ProtectedAreaSize > 0) {
        printf("  ?? Protected Area: %lu sectors configured\r\n",
               status->ProtectedAreaSize);
    }
}

extern SD_HandleTypeDef hsd2;

void print_sd_card_details(void) {
    HAL_SD_CardCSDTypeDef csd;
    HAL_SD_CardCIDTypeDef cid;
    HAL_SD_CardStatusTypeDef cstat;



    if (HAL_SD_GetCardCSD(&hsd2, &csd) == HAL_OK &&
        HAL_SD_GetCardCID(&hsd2, &cid) == HAL_OK) {

        print_sd_card_info(&csd, &cid);
    } else {
        printf("Error: Could not read SD card registers\r\n");
    }

    HAL_SD_GetCardStatus(&hsd2,&cstat);
    print_sd_card_status(&cstat);       // New status function
    printf("cstat = %d\n",cstat.DataBusWidth);
}
