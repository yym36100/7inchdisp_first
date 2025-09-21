/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

LTDC_HandleTypeDef hltdc;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_I2C4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sdram_init(void){
	FMC_SDRAM_CommandTypeDef command;

	// Step 1: Enable clock
	command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
	command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
	command.AutoRefreshNumber = 1;
	command.ModeRegisterDefinition = 0;
	HAL_SDRAM_SendCommand(&hsdram1, &command, 0x1000);
	HAL_Delay(1); // >100 µs

	// Step 2: Precharge all
	command.CommandMode = FMC_SDRAM_CMD_PALL;
	HAL_SDRAM_SendCommand(&hsdram1, &command, 0x1000);

	// Step 3: Auto-refresh (2 cycles)
	command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	command.AutoRefreshNumber = 2;
	HAL_SDRAM_SendCommand(&hsdram1, &command, 0x1000);

	// Step 4: Load Mode Register
	uint32_t mode_reg = 0
	  | (0x0 << 0)  // Burst Length = 1
	  | (0x0 << 3)  // Burst Type = Sequential
	  | (0x3 << 4)  // CAS Latency = 3
	  | (0x0 << 7)  // Standard Operation
	  | (0x1 << 9); // Write burst = Single

	command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	command.ModeRegisterDefinition = mode_reg;
	HAL_SDRAM_SendCommand(&hsdram1, &command, 0x1000);

	// Step 5: Set refresh rate
	HAL_SDRAM_ProgramRefreshRate(&hsdram1, 780);

}

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


#define SDRAM_START ((uint32_t)0xC0000000)   // SDRAM Bank1 base
#define SDRAM_SIZE  ((uint32_t)0x02000000)   // 32 MB for W9825G6KH (16M x16)
#define SDRAM_END   (SDRAM_START + SDRAM_SIZE)

static inline uint32_t *sdram_ptr(uint32_t offset) {
    return (uint32_t *)(SDRAM_START + offset);
}

int sdram_memtest(void)
{
    uint32_t errors = 0;
    uint32_t addr;
    uint32_t readback;

    printf("SDRAM test: 0x%08lX .. 0x%08lX\n", (unsigned long)SDRAM_START, (unsigned long)SDRAM_END-1);

    // 1. Walking bit test
    for (uint32_t bit = 0; bit < 32; bit++) {
        uint32_t pattern = 1UL << bit;
        *sdram_ptr(0) = pattern;
        readback = *sdram_ptr(0);
        if (readback != pattern) {
            printf("Walking bit error at bit %lu: wrote 0x%08lX, read 0x%08lX\n",
                   (unsigned long)bit, (unsigned long)pattern, (unsigned long)readback);
            errors++;
        }
    }

    // 2. Address test
    for (addr = 0; addr < SDRAM_SIZE; addr += 4) {
        *sdram_ptr(addr) = addr ^ 0xAAAAAAAA;
    }
    for (addr = 0; addr < SDRAM_SIZE; addr += 4) {
        readback = *sdram_ptr(addr);
        if (readback != (addr ^ 0xAAAAAAAA)) {
            printf("Address test error at 0x%08lX: wrote 0x%08lX, read 0x%08lX\n",
                   (unsigned long)(SDRAM_START+addr), (unsigned long)(addr ^ 0xAAAAAAAA), (unsigned long)readback);
            errors++;
            break;
        }
    }

    // 3. Pattern test
    const uint32_t patterns[] = { 0x00000000, 0xFFFFFFFF, 0xAAAAAAAA, 0x55555555, 0x12345678, 0x87654321 };
    for (int p = 0; p < (int)(sizeof(patterns)/sizeof(patterns[0])); p++) {
        uint32_t pat = patterns[p];
        printf("Pattern 0x%08lX ... ", (unsigned long)pat);

        for (addr = 0; addr < SDRAM_SIZE; addr += 4) {
            *sdram_ptr(addr) = pat;
        }
        for (addr = 0; addr < SDRAM_SIZE; addr += 4) {
            readback = *sdram_ptr(addr);
            if (readback != pat) {
                printf("\nPattern error at 0x%08lX: wrote 0x%08lX, read 0x%08lX\n",
                       (unsigned long)(SDRAM_START+addr), (unsigned long)pat, (unsigned long)readback);
                errors++;
                break;
            }
        }
        printf("done\n");
    }

    if (errors == 0) {
        printf("SDRAM test passed.\n");
        return 0;
    } else {
        printf("SDRAM test failed with %lu errors.\n", (unsigned long)errors);
        return -1;
    }
}


#define SDRAM_BASE_ADDR 0xC0000000
#undef SDRAM_SIZE
#define SDRAM_SIZE      (32*1024*1024) // 32 MB
#define SDRAM_WORDS     (SDRAM_SIZE / 4)

typedef enum {
    TEST_PATTERN_FIXED,
    TEST_PATTERN_WALKING_ONES,
    TEST_PATTERN_WALKING_ZEROS,
    TEST_PATTERN_ALTERNATING,
    TEST_PATTERN_RANDOM
} TestPattern;

static inline uint32_t prng(uint32_t *state) {
    // simple 32-bit LCG pseudo-random generator
    *state = *state * 1664525 + 1013904223;
    return *state;
}

// Perform one pass with a single pattern
int SDRAM_Test_Pattern(uint32_t *mem, TestPattern pattern)
{
    uint32_t state = 0x12345678;
    uint32_t write_val = 0;

    for(uint32_t i = 0; i < SDRAM_WORDS; i++)
    {
        switch(pattern)
        {
            case TEST_PATTERN_FIXED: write_val = 0xA5A5A5A5; break;
            case TEST_PATTERN_WALKING_ONES: write_val = 1U << (i % 32); break;
            case TEST_PATTERN_WALKING_ZEROS: write_val = ~(1U << (i % 32)); break;
            case TEST_PATTERN_ALTERNATING: write_val = (i % 2) ? 0xAAAAAAAA : 0x55555555; break;
            case TEST_PATTERN_RANDOM: write_val = prng(&state); break;
        }
        mem[i] = write_val;
    }

    // Ensure all writes are visible in D-Cache
    SCB_CleanDCache_by_Addr((uint32_t*)mem, SDRAM_SIZE);
    state = 0x12345678;

    for(uint32_t i = 0; i < SDRAM_WORDS; i++)
    {
        switch(pattern)
        {
            case TEST_PATTERN_FIXED: write_val = 0xA5A5A5A5; break;
            case TEST_PATTERN_WALKING_ONES: write_val = 1U << (i % 32); break;
            case TEST_PATTERN_WALKING_ZEROS: write_val = ~(1U << (i % 32)); break;
            case TEST_PATTERN_ALTERNATING: write_val = (i % 2) ? 0xAAAAAAAA : 0x55555555; break;
            case TEST_PATTERN_RANDOM: write_val = prng(&state); break;
        }
        uint32_t read_val = mem[i];
        SCB_InvalidateDCache_by_Addr((uint32_t*)&mem[i], 4);
        if(read_val != write_val)
        {
            return i; // first failing index
        }
    }

    return -1; // passed
}

// Full SDRAM memory test with multiple patterns
void SDRAM_FullTest(void)
{
    uint32_t *mem = (uint32_t*)SDRAM_BASE_ADDR;
    int res;

    TestPattern patterns[] = {
        TEST_PATTERN_FIXED,
        TEST_PATTERN_WALKING_ONES,
        TEST_PATTERN_WALKING_ZEROS,
        TEST_PATTERN_ALTERNATING,
        TEST_PATTERN_RANDOM
    };

    for(int p = 0; p < sizeof(patterns)/sizeof(patterns[0]); p++)
    {
        res = SDRAM_Test_Pattern(mem, patterns[p]);
        if(res < 0)
            printf("Pattern %d passed\n", p);
        else
            printf("Pattern %d failed at address 0x%08X\n", p, SDRAM_BASE_ADDR + res*4);
    }
}



typedef enum {
    PATTERN_FIXED,
    PATTERN_WALKING_ONES,
    PATTERN_WALKING_ZEROS,
    PATTERN_ALTERNATING,
    PATTERN_RANDOM
} TestPattern2;

typedef enum {
    ACCESS_8BIT=8,
    ACCESS_16BIT=16,
    ACCESS_32BIT=32
} AccessSize;


// Fill SDRAM with a pattern
void SDRAM_Fill(uint32_t base, TestPattern2 pattern, AccessSize size, uint32_t seed)
{
    uint32_t state = seed;
    for(uint32_t offset = 0; offset < SDRAM_SIZE; offset += (size/8))
    {
        uint32_t val = 0;
        switch(pattern)
        {
            case PATTERN_FIXED: val = 0xA5A5A5A5; break;
            case PATTERN_WALKING_ONES: val = 1U << ((offset/size*8) % 32); break;
            case PATTERN_WALKING_ZEROS: val = ~(1U << ((offset/size*8) % 32)); break;
            case PATTERN_ALTERNATING: val = ((offset/(size/8)) %2) ? 0xAAAAAAAA : 0x55555555; break;
            case PATTERN_RANDOM: val = prng(&state); break;
        }

        switch(size)
        {
            case ACCESS_8BIT:  *((volatile uint8_t *)(base + offset)) = (uint8_t)val; break;
            case ACCESS_16BIT: *((volatile uint16_t*)(base + offset)) = (uint16_t)val; break;
            case ACCESS_32BIT: *((volatile uint32_t*)(base + offset)) = val; break;
        }
    }

    // Ensure all writes hit SDRAM
    SCB_CleanDCache_by_Addr((uint32_t*)base, SDRAM_SIZE);
}

// Verify SDRAM pattern
int SDRAM_Verify(uint32_t base, TestPattern2 pattern, AccessSize size, uint32_t seed)
{
    uint32_t state = seed;
    for(uint32_t offset = 0; offset < SDRAM_SIZE; offset += (size/8))
    {
        uint32_t expected = 0;
        switch(pattern)
        {
            case PATTERN_FIXED: expected = 0xA5A5A5A5; break;
            case PATTERN_WALKING_ONES: expected = 1U << ((offset/size*8) % 32); break;
            case PATTERN_WALKING_ZEROS: expected = ~(1U << ((offset/size*8) % 32)); break;
            case PATTERN_ALTERNATING: expected = ((offset/(size/8)) %2) ? 0xAAAAAAAA : 0x55555555; break;
            case PATTERN_RANDOM: expected = prng(&state); break;
        }

        uint32_t read_val = 0;
        switch(size)
        {
            case ACCESS_8BIT:  read_val = *((volatile uint8_t *)(base + offset)); break;
            case ACCESS_16BIT: read_val = *((volatile uint16_t*)(base + offset)); break;
            case ACCESS_32BIT: read_val = *((volatile uint32_t*)(base + offset)); break;
        }

        SCB_InvalidateDCache_by_Addr((uint32_t*)(base + offset), size/8);

        if(read_val != (expected & ((1UL<<size)-1)))
        {
            return offset; // first failing address
        }
    }
    return -1; // passed
}

// Run full SDRAM test
void SDRAM_FullTest2(void)
{
    TestPattern2 patterns[] = {PATTERN_FIXED, PATTERN_WALKING_ONES, PATTERN_WALKING_ZEROS, PATTERN_ALTERNATING, PATTERN_RANDOM};
    AccessSize sizes[] = {ACCESS_8BIT, ACCESS_16BIT, ACCESS_32BIT};
    uint32_t seed = 0x12345678;

    for(int p=0; p<sizeof(patterns)/sizeof(patterns[0]); p++)
    {
        for(int s=0; s<sizeof(sizes)/sizeof(sizes[0]); s++)
        {
            SDRAM_Fill(SDRAM_BASE_ADDR, patterns[p], sizes[s], seed);
            int fail = SDRAM_Verify(SDRAM_BASE_ADDR, patterns[p], sizes[s], seed);
            if(fail < 0)
                printf("Pattern %d, Access %d-bit: PASS\n", p, (s==0?8:(s==1?16:32)));
            else
                printf("Pattern %d, Access %d-bit: FAIL at 0x%08X\n", p, (s==0?8:(s==1?16:32)), SDRAM_BASE_ADDR+fail);
        }
    }
}


extern void run_memory_benchmarks(void);
extern void run_dma_benchmarks(void);


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UART_Transmit(&huart1, "Start\n", 6, 100);
  printf("Start\n");

  sdram_init();
  printf("SDRAM inited\n");
  printf("Starting memtest\n");
  {
	  volatile uint32_t *p = (uint32_t*)0xC0000000;
	  for(uint32_t i = 0; i < 32*1024*1024/4; i++) {
	      p[i] = 0xA5A5A5A5;      // write pattern
	      if(p[i] != 0xA5A5A5A5) {
	          // report fault, address = &p[i]
	    	  printf("fault at %08x:%04x\n",&p[i],p[i]);
	          break;
	      }
	  }

  }
  //sdram_memtest();
  //SDRAM_FullTest();
  //SDRAM_FullTest2();
  HAL_GPIO_WritePin(lcd_rst_GPIO_Port, lcd_rst_Pin, 1);
  run_memory_benchmarks();
  run_dma_benchmarks();
  __HAL_LTDC_ENABLE(&hltdc);
  __HAL_LTDC_RELOAD_CONFIG(&hltdc); // reload shadow registers
  __HAL_LTDC_LAYER_ENABLE(&hltdc,0);
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port,LCD_BL_Pin,1);
  //memset(0xc0000000,0xf0,1024*600*1);
	{
		uint32_t *src_sdram = (uint16_t*) 0xC0000000;
		for (int i = 0; i < 1024 * 600; i++) {
			*src_sdram++ = i;
		}
	}
  SCB_CleanDCache_by_Addr((uint32_t*)0xc0000000, 1024*600*2);
  //restore c:\Users\yym\Downloads\sijeles.bin  binary 0xC0000000


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Transmit(&huart1, ".", 1, 100);
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10C0ECFF;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 4;
  hltdc.Init.AccumulatedHBP = 49;
  hltdc.Init.AccumulatedVBP = 24;
  hltdc.Init.AccumulatedActiveW = 1073;
  hltdc.Init.AccumulatedActiveH = 624;
  hltdc.Init.TotalWidth = 1104;
  hltdc.Init.TotalHeigh = 634;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 255;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 1024;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 600;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0xc0000000;
  pLayerCfg.ImageWidth = 1024;
  pLayerCfg.ImageHeight = 600;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 255;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 5;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(lcd_rst_GPIO_Port, lcd_rst_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GT_RST_GPIO_Port, GT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : lcd_rst_Pin */
  GPIO_InitStruct.Pin = lcd_rst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(lcd_rst_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : gt_int_Pin */
  GPIO_InitStruct.Pin = gt_int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(gt_int_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin GT_RST_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|GT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0xc0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
