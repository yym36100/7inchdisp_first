#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>

#pragma GCC push_options
#pragma GCC optimize ("O3")   // or "Os", "O2", etc.

uint32_t internalrambuff[256*1024/4];

typedef enum { ACCESS_8BIT = 8,  ACCESS_16BIT= 16, ACCESS_32BIT = 32 } AccessSize;

typedef struct {
    uint32_t read_bytes;
    uint32_t write_bytes;
    float read_MBps;
    float write_MBps;
} MemBenchmarkResult;

// --- DWT Cycle Counter ---
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

// --- Generic memory benchmark ---
MemBenchmarkResult benchmark_memory(void *base, uint32_t size_bytes, AccessSize width)
{
    MemBenchmarkResult res = {0};
    volatile uint32_t dummy_sum = 0; // prevent optimization

    uint32_t count = size_bytes / (width / 8);

    uint8_t  *ptr8  = (uint8_t*)base;
    uint16_t *ptr16 = (uint16_t*)base;
    uint32_t *ptr32 = (uint32_t*)base;

    // --- Write Benchmark ---
    SCB_InvalidateDCache_by_Addr((uint32_t*)base, size_bytes); // invalidate before writes
    uint32_t start = DWT_GetCycles();
    uint32_t tst = HAL_GetTick();
    for(uint32_t i=0;i<count;i++)
    {
        switch(width)
        {
            case 8:  ptr8[i]  = 0xA5; break;
            case 16: ptr16[i] = 0xA5A5; break;
            case 32: ptr32[i] = 0xA5A5A5A5; break;
        }
    }
    SCB_CleanDCache_by_Addr((uint32_t*)base, size_bytes);
    uint32_t end = DWT_GetCycles();
    uint32_t tsp = HAL_GetTick();
    uint32_t hclock = HAL_RCC_GetSysClockFreq();

    res.write_bytes = size_bytes;
    res.write_MBps = ((float)size_bytes / 1e6f) / ((float)(end-start) / (float)hclock);

    printf("start=%d, stop=%d hclock = %d\n",tst, tsp,hclock);

    // --- Read Benchmark ---
    SCB_InvalidateDCache_by_Addr((uint32_t*)base, size_bytes); // force actual memory read
    start = DWT_GetCycles();
    for(uint32_t i=0;i<count;i++)
    {
        switch(width)
        {
            case 8:  dummy_sum += ptr8[i]; break;
            case 16: dummy_sum += ptr16[i]; break;
            case 32: dummy_sum += ptr32[i]; break;
        }
    }
    end = DWT_GetCycles();
    res.read_bytes = size_bytes;
    res.read_MBps = ((float)size_bytes / 1e6f) / ((float)(end-start) / HAL_RCC_GetHCLKFreq());

    // use dummy_sum so compiler won't optimize away reads
    if(dummy_sum == 0xFFFFFFFF) printf("Impossible\n");

    return res;
}

// --- Example usage ---
void run_memory_benchmarks(void)
{
    DWT_Init();

    // Test SDRAM 32MB
    MemBenchmarkResult sdram32 = benchmark_memory((void*)0xC0000000, 32*1024*1024, ACCESS_32BIT);
    printf("SDRAM 32-bit: Read %.1f MB/s, Write %.1f MB/s\n", sdram32.read_MBps, sdram32.write_MBps);

    MemBenchmarkResult sdram16 = benchmark_memory((void*)0xC0000000, 32*1024*1024, ACCESS_16BIT);
        printf("SDRAM 16-bit: Read %.1f MB/s, Write %.1f MB/s\n", sdram16.read_MBps, sdram16.write_MBps);

        MemBenchmarkResult sdram8 = benchmark_memory((void*)0xC0000000, 32*1024*1024, ACCESS_8BIT);
            printf("SDRAM 8-bit: Read %.1f MB/s, Write %.1f MB/s\n", sdram8.read_MBps, sdram8.write_MBps);

    // Test internal SRAM 512KB
    MemBenchmarkResult sram32 = benchmark_memory((void*)internalrambuff, 256*1024, ACCESS_32BIT);
    printf("SRAM 32-bit: Read %.1f MB/s, Write %.1f MB/s\n", sram32.read_MBps, sram32.write_MBps);

    // Test internal Flash 1MB
    MemBenchmarkResult flash32 = benchmark_memory((void*)0x08000000, 1024*1024, ACCESS_32BIT);
    printf("FLASH 32-bit: Read %.1f MB/s\n", flash32.read_MBps); // skip writes for flash

    // Optionally: QSPI external flash
#if 0
    MemBenchmarkResult qspi32 = benchmark_memory((void*)QSPI_BASE, 8*1024*1024, ACCESS_32BIT);
    printf("QSPI 32-bit: Read %.1f MB/s\n", qspi32.read_MBps);
#endif
}

#pragma GCC pop_options
