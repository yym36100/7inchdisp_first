#include <stdio.h>
#include <stdint.h>


#include "main.h"

// Utility to extract arbitrary bits from 4x32-bit array
uint32_t extract_bits(uint32_t csd[4], int start_bit, int size)
{
    uint32_t value = 0;
    for (int i = 0; i < size; i++) {
        int bit = start_bit - i; // iterate from MSB down
        int word_index = 3 - (bit / 32);
        int bit_in_word = bit % 32;
        uint32_t bit_val = (csd[word_index] >> bit_in_word) & 1;
        value = (value << 1) | bit_val;
    }
    return value;
}


void decode_csd_full(uint32_t csd[4]) {
    uint32_t csd_structure = extract_bits(csd, 126, 2);
    printf("CSD_STRUCTURE: %u\n", csd_structure);

    printf("TAAC: 0x%02X\n", extract_bits(csd, 112, 8));
    printf("NSAC: 0x%02X\n", extract_bits(csd, 104, 8));
    printf("TRAN_SPEED: 0x%02X\n", extract_bits(csd, 96, 8));
    printf("CCC: 0x%03X\n", extract_bits(csd, 84, 12));
    printf("READ_BL_LEN: %u\n", extract_bits(csd, 80, 4));
    printf("READ_BL_PARTIAL: %u\n", extract_bits(csd, 79, 1));
    printf("WRITE_BLK_MISALIGN: %u\n", extract_bits(csd, 78, 1));
    printf("READ_BLK_MISALIGN: %u\n", extract_bits(csd, 77, 1));
    printf("DSR_IMP: %u\n", extract_bits(csd, 76, 1));

    if (csd_structure == 0) { // CSD v1
        uint32_t c_size = extract_bits(csd, 62, 12);
        uint32_t c_size_mult = extract_bits(csd, 47, 3);
        uint32_t read_bl_len = extract_bits(csd, 80, 4);
        uint32_t card_capacity = (c_size + 1) * (1 << (c_size_mult + 2)) * (1 << read_bl_len);
        printf("C_SIZE: %u\n", c_size);
        printf("C_SIZE_MULT: %u\n", c_size_mult);
        printf("CARD_CAPACITY: %u bytes (~%u MB)\n", card_capacity, card_capacity / 1024 / 1024);
    } else if (csd_structure == 1) { // CSD v2
        uint32_t c_size = extract_bits(csd, 48, 22);
        uint32_t card_capacity = (c_size + 1) * 512UL * 1024UL;
        printf("C_SIZE: %u\n", c_size);
        printf("CARD_CAPACITY: %lu bytes (~%lu MB)\n", card_capacity, card_capacity / 1024 / 1024);
    }

    printf("ERASE_BLK_EN: %u\n", extract_bits(csd, 46, 1));
    printf("SECTOR_SIZE: %u\n", extract_bits(csd, 39, 7));
    printf("WP_GRP_SIZE: %u\n", extract_bits(csd, 32, 7));
    printf("WP_GRP_ENABLE: %u\n", extract_bits(csd, 31, 1));
    printf("R2W_FACTOR: %u\n", extract_bits(csd, 26, 3));
    printf("WRITE_BL_LEN: %u\n", extract_bits(csd, 22, 4));
    printf("WRITE_BL_PARTIAL: %u\n", extract_bits(csd, 21, 1));
    printf("FILE_FORMAT_GRP: %u\n", extract_bits(csd, 15, 1));
    printf("COPY: %u\n", extract_bits(csd, 14, 1));
    printf("PERM_WRITE_PROTECT: %u\n", extract_bits(csd, 13, 1));
    printf("TMP_WRITE_PROTECT: %u\n", extract_bits(csd, 12, 1));
    printf("FILE_FORMAT: %u\n", extract_bits(csd, 10, 2));
    printf("CRC: %u\n", extract_bits(csd, 1, 7));
}

void decode_cid_full(uint32_t cid[4]) {
    printf("MID: 0x%02X\n", (cid[0] >> 24) & 0xFF);
    printf("OID: %c%c\n", (cid[0] >> 16) & 0xFF, (cid[0] >> 8) & 0xFF);
    printf("PNM: %c%c%c%c%c\n", (cid[0]&0xFF), (cid[1]>>24)&0xFF, (cid[1]>>16)&0xFF,
                                 (cid[1]>>8)&0xFF, cid[1]&0xFF);
    printf("PRV: %u.%u\n", (cid[2]>>24)&0xF, (cid[2]>>20)&0xF);
    printf("PSN: 0x%08X\n", cid[2]&0xFFFFF0 | (cid[3]>>28)&0xF);
    printf("MDT: %u/%u\n", (cid[3]>>8)&0xF, 2000 + ((cid[3]>>12)&0xFF));
    printf("CRC: %u\n", (cid[3]>>1)&0x7F);
}

extern SD_HandleTypeDef hsd2;



// Extract bits [start:start+size-1] from a 128-bit big-endian response
static inline uint32_t unstuff_bits(const uint32_t *resp, int start, int size)
{
    int off = 3 - (start / 32);   // MSB first
    int shift = start % 32;
    uint32_t val = resp[off] >> ( shift - size);
    if (shift + size > 32)
        val |= resp[off - 1] << (shift + size - 32);
    return val & ((1ULL << size) - 1);
}

void decode_csd(const uint32_t csd[4])
{
    printf("=== CSD ===\n");

    uint8_t  csd_structure   = unstuff_bits(csd, 126, 2);
    uint8_t  taac            = unstuff_bits(csd, 112, 8);
    uint8_t  nsac            = unstuff_bits(csd, 104, 8);
    uint8_t  tran_speed      = unstuff_bits(csd, 96, 8);
    uint16_t ccc             = unstuff_bits(csd, 84, 12);
    uint8_t  read_bl_len     = unstuff_bits(csd, 80, 4);
    uint8_t  read_bl_partial = unstuff_bits(csd, 79, 1);
    uint8_t  write_blk_misalign = unstuff_bits(csd, 78, 1);
    uint8_t  read_blk_misalign  = unstuff_bits(csd, 77, 1);
    uint8_t  dsr_imp         = unstuff_bits(csd, 76, 1);
    uint16_t c_size;
    uint8_t  c_size_mult;
    uint32_t capacity;

    if (csd_structure == 0) {
        // CSD Version 1.0 (Standard Capacity)
        c_size      = unstuff_bits(csd, 62, 12);
        c_size_mult = unstuff_bits(csd, 47, 3);
        capacity    = (c_size + 1) << (c_size_mult + 2 + read_bl_len);
    } else if (csd_structure == 1) {
        // CSD Version 2.0 (High Capacity)
        c_size      = unstuff_bits(csd, 48, 22);
        capacity    = (c_size + 1) * 512 * 1024; // always 512-byte blocks
    } else {
        printf("Unknown CSD structure %d\n", csd_structure);
        return;
    }

    uint8_t erase_blk_en     = unstuff_bits(csd, 46, 1);
    uint8_t sector_size      = unstuff_bits(csd, 39, 7);
    uint8_t wp_grp_size      = unstuff_bits(csd, 32, 7);
    uint8_t wp_grp_enable    = unstuff_bits(csd, 31, 1);
    uint8_t r2w_factor       = unstuff_bits(csd, 26, 3);
    uint8_t write_bl_len     = unstuff_bits(csd, 22, 4);
    uint8_t write_bl_partial = unstuff_bits(csd, 21, 1);
    uint8_t file_format_grp  = unstuff_bits(csd, 15, 1);
    uint8_t copy             = unstuff_bits(csd, 14, 1);
    uint8_t perm_write_protect = unstuff_bits(csd, 13, 1);
    uint8_t tmp_write_protect  = unstuff_bits(csd, 12, 1);
    uint8_t file_format      = unstuff_bits(csd, 10, 2);
    uint8_t crc              = unstuff_bits(csd, 1, 7);

    // Print results
    printf("CSD_STRUCTURE: %d\n", csd_structure);
    printf("TAAC: 0x%02X\n", taac);
    printf("NSAC: 0x%02X\n", nsac);
    printf("TRAN_SPEED: 0x%02X\n", tran_speed);
    printf("CCC: 0x%03X\n", ccc);
    printf("READ_BL_LEN: %d\n", read_bl_len);
    printf("READ_BL_PARTIAL: %d\n", read_bl_partial);
    printf("WRITE_BLK_MISALIGN: %d\n", write_blk_misalign);
    printf("READ_BLK_MISALIGN: %d\n", read_blk_misalign);
    printf("DSR_IMP: %d\n", dsr_imp);
    printf("C_SIZE: %u\n", c_size);
    if (csd_structure == 0)
        printf("C_SIZE_MULT: %u\n", c_size_mult);
    printf("CARD_CAPACITY: %u bytes (~%u MB)\n", capacity, capacity / (1024*1024));
    printf("ERASE_BLK_EN: %d\n", erase_blk_en);
    printf("SECTOR_SIZE: %d\n", sector_size);
    printf("WP_GRP_SIZE: %d\n", wp_grp_size);
    printf("WP_GRP_ENABLE: %d\n", wp_grp_enable);
    printf("R2W_FACTOR: %d\n", r2w_factor);
    printf("WRITE_BL_LEN: %d\n", write_bl_len);
    printf("WRITE_BL_PARTIAL: %d\n", write_bl_partial);
    printf("FILE_FORMAT_GRP: %d\n", file_format_grp);
    printf("COPY: %d\n", copy);
    printf("PERM_WRITE_PROTECT: %d\n", perm_write_protect);
    printf("TMP_WRITE_PROTECT: %d\n", tmp_write_protect);
    printf("FILE_FORMAT: %d\n", file_format);
    printf("CRC: %d\n", crc);
}


static uint32_t extract2(const uint32_t csd[4], int start, int size)
{
    // start = bit index (127 = MSB of csd[0], 0 = LSB of csd[3])
    uint64_t val = 0;
    for (int i = 0; i < 128; i++) {
        int word = i / 32;
        int bit  = 31 - (i % 32);
        int src_bit = (csd[word] >> bit) & 1;

        if (i >= start && i < start + size) {
            val = (val << 1) | src_bit;
        }
    }
    return (uint32_t)val;
}

void decode_csd2(const uint32_t csd[4])
{
    printf("CSD_STRUCTURE: %u\n", extract2(csd, 126, 2));
    printf("TAAC: 0x%02X\n", extract2(csd, 112, 8));
    printf("NSAC: 0x%02X\n", extract2(csd, 104, 8));
    printf("TRAN_SPEED: 0x%02X\n", extract2(csd, 96, 8));
    printf("CCC: 0x%03X\n", extract2(csd, 84, 12));
    printf("READ_BL_LEN: %u\n", extract2(csd, 80, 4));
    // …continue for all fields per SD spec…
}

int print_sd_info() {
    uint32_t csd[4] = {0x007f0032, 0x535a801d, 0xeebbff9f, 0x1680008e};
    //uint32_t cid[4] = {0x1b534d30, 0x30303101, 0x5e00abcd, 0x0f123456};

    printf("=== CSD ===\n");
    decode_csd_full(csd);
    printf("=== CSD2 ===\n");
    decode_csd2(csd);

    printf("\n=== CID ===\n");
    decode_cid_full(hsd2.CID);

    printf("linux version\n");
    decode_csd(csd);

    return 0;
}
