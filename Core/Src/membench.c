#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>

#pragma GCC push_options
#pragma GCC optimize ("O3")   // or "Os", "O2", etc.

uint32_t internalrambuff[256*1024/4];

uint32_t dtcram[32*1024/4];


#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>

typedef enum { ACCESS_8BIT = 8, ACCESS_16BIT = 16, ACCESS_32BIT = 32 } AccessSize;

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

// --- Write 32-bit benchmark safe from memset optimization ---
static inline void write32(volatile uint32_t *ptr, uint32_t count)
{
    for(uint32_t i=0;i<count;i++)
    {
        ptr[i] = i; // unique pattern prevents memset
        __asm__ volatile("" ::: "memory"); // compiler barrier
    }
}

// --- Read 32-bit benchmark ---
static inline uint32_t read32(volatile uint32_t *ptr, uint32_t count)
{
    uint32_t sum = 0;
    for(uint32_t i=0;i<count;i++)
    {
        sum += ptr[i];
        __asm__ volatile("" ::: "memory"); // prevent optimization
    }
    return sum;
}

// --- Generic memory benchmark ---
MemBenchmarkResult benchmark_memory32(void *base, uint32_t size_bytes)
{
    MemBenchmarkResult res = {0};
    volatile uint32_t *ptr = (volatile uint32_t*)base;
    uint32_t count = size_bytes / 4;

    // --- Write benchmark ---
    SCB_InvalidateDCache_by_Addr((uint32_t*)base, size_bytes);
    uint32_t start = DWT_GetCycles();
    write32(ptr, count);
    SCB_CleanDCache_by_Addr((uint32_t*)base, size_bytes);
    uint32_t end = DWT_GetCycles();

    res.write_bytes = size_bytes;
    res.write_MBps = ((float)size_bytes / 1e6f) / ((float)(end-start) / HAL_RCC_GetSysClockFreq());

    // --- Read benchmark ---
    SCB_InvalidateDCache_by_Addr((uint32_t*)base, size_bytes);
    start = DWT_GetCycles();
    uint32_t sum = read32(ptr, count);
    end = DWT_GetCycles();

    res.read_bytes = size_bytes;
    res.read_MBps = ((float)size_bytes / 1e6f) / ((float)(end-start) / HAL_RCC_GetSysClockFreq());

    // Use sum to prevent compiler optimizing away reads
    if(sum == 0xFFFFFFFF) printf("Impossible\n");

    return res;
}

// --- Example usage ---
void run_memory_benchmarks(void)
{
    DWT_Init();

    // 1. Internal SRAM 512KB
    MemBenchmarkResult sram32 = benchmark_memory32((void*)internalrambuff, 256*1024);
    printf("SRAM 32-bit: Read %.1f MB/s, Write %.1f MB/s\n", sram32.read_MBps, sram32.write_MBps);


    // 1. Internal SRAM 512KB
       sram32 = benchmark_memory32((void*)dtcram, 32*1024);
       printf("dtcram 32-bit: Read %.1f MB/s, Write %.1f MB/s\n", sram32.read_MBps, sram32.write_MBps);


    // 2. SDRAM 32MB
    MemBenchmarkResult sdram32 = benchmark_memory32((void*)0xC0000000, 32*1024*1024);
    printf("SDRAM 32-bit: Read %.1f MB/s, Write %.1f MB/s\n", sdram32.read_MBps, sdram32.write_MBps);

    // 3. Internal Flash 1MB (read only)
    MemBenchmarkResult flash32 = benchmark_memory32((void*)0x08000000, 1024*1024);
    printf("FLASH 32-bit: Read %.1f MB/s\n", flash32.read_MBps);

#if 0
    // 4. Optional: QSPI external flash
    MemBenchmarkResult qspi32 = benchmark_memory32((void*)QSPI_BASE, 8*1024*1024);
    printf("QSPI 32-bit: Read %.1f MB/s\n", qspi32.read_MBps);
#endif
}




DMA_HandleTypeDef hdma_memtomem;

// Initialize DMA1 Stream 0 for memory-to-memory transfers
void DMA_MemToMem_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_memtomem.Instance = DMA1_Stream0;
    hdma_memtomem.Init.Request = DMA_REQUEST_MEM2MEM;
    hdma_memtomem.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem.Init.PeriphInc = DMA_PINC_ENABLE;
    hdma_memtomem.Init.MemInc = DMA_MINC_ENABLE;
    hdma_memtomem.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; // 32-bit
    hdma_memtomem.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_memtomem.Init.Mode = DMA_NORMAL;
    hdma_memtomem.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_memtomem.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_memtomem.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem.Init.MemBurst = DMA_MBURST_INC4;
    hdma_memtomem.Init.PeriphBurst = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdma_memtomem);
}

MemBenchmarkResult benchmark_memory_dma32(void *src, void *dst, uint32_t size_bytes)
{
    MemBenchmarkResult res = {0};

    // Ensure cache coherence
    SCB_CleanDCache_by_Addr((uint32_t*)src, size_bytes);
    SCB_InvalidateDCache_by_Addr((uint32_t*)dst, size_bytes);

    uint32_t start = DWT_GetCycles();

    // Start DMA transfer
    HAL_DMA_Start(&hdma_memtomem, (uint32_t)src, (uint32_t)dst, size_bytes/4);
    HAL_DMA_PollForTransfer(&hdma_memtomem, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);

    uint32_t end = DWT_GetCycles();

    res.read_bytes = size_bytes;
    res.write_bytes = size_bytes;
    res.read_MBps = ((float)size_bytes / 1e6f) / ((float)(end-start) / HAL_RCC_GetSysClockFreq());
    res.write_MBps = res.read_MBps;

    return res;
}

static uint32_t src_sram[32*1024/4];  // 512KB
static uint32_t dst_sram[32*1024/4];
void run_dma_benchmarks(void)
{
    DWT_Init();
    DMA_MemToMem_Init();



    MemBenchmarkResult sram_dma = benchmark_memory_dma32(src_sram, dst_sram, sizeof(src_sram));
    printf("SRAM DMA 32-bit: %.1f MB/s\n", sram_dma.read_MBps);

    static uint32_t *src_sdram = (uint32_t*)0xC0000000;
    static uint32_t *dst_sdram = (uint32_t*)0xC0080000; // offset to avoid overlap

    MemBenchmarkResult sdram_dma = benchmark_memory_dma32(src_sram, dst_sdram, 32*1024);
    printf("SDRAM DMA write 32-bit: %.1f MB/s\n", sdram_dma.read_MBps);

     sdram_dma = benchmark_memory_dma32(src_sdram, dst_sram, 32*1024);
    printf("SDRAM DMA read 32-bit: %.1f MB/s\n", sdram_dma.read_MBps);

#if  0
    static uint32_t *src_qspi = (uint32_t*)QSPI_BASE;
    static uint32_t dst_qspi[128*1024];
    MemBenchmarkResult qspi_dma = benchmark_memory_dma32(src_qspi, dst_qspi, 512*1024);
    printf("QSPI DMA 32-bit: %.1f MB/s\n", qspi_dma.read_MBps);
#endif
}



#pragma GCC pop_options
