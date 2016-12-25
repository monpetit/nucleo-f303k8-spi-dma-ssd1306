/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "binary.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile bool uart2_transmit_complete = false;
volatile bool timer7_flag = false;
volatile bool spi1_transmit_complete = false;

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

static const unsigned char /*PROGMEM*/ logo16_glcd_bmp[] = {
    B00000000, B11000000,
    B00000001, B11000000,
    B00000001, B11000000,
    B00000011, B11100000,
    B11110011, B11100000,
    B11111110, B11111000,
    B01111110, B11111111,
    B00110011, B10011111,
    B00011111, B11111100,
    B00001101, B01110000,
    B00011011, B10100000,
    B00111111, B11100000,
    B00111111, B11110000,
    B01111100, B11110000,
    B01110000, B01110000,
    B00000000, B00110000
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void testdrawchar(void);
void testdrawline(void);
void testdrawbitmap(const uint8_t *bitmap, uint8_t w, uint8_t h);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static char stxbuff[128];
#define printf(...)                                                            \
    do {                                                                       \
        uart2_transmit_complete = false;                                       \
        sprintf(stxbuff, __VA_ARGS__);                                         \
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)stxbuff, strlen(stxbuff));   \
        while (!uart2_transmit_complete) __WFE();                              \
    } while (0)

#define uart_write_buffer(BUFFER, LENGTH)                                      \
    do {                                                                       \
        uart2_transmit_complete = false;                                       \
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BUFFER, LENGTH);             \
        while (!uart2_transmit_complete) __WFE();                              \
    } while (0)

#define DELAY_MS 1000 /**< Timer Delay in milli-seconds. */
#define DRIVER_SSD1306_I2C       1

/* USER CODE END 0 */

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_TIM7_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim7);
    printf("hello...\r\n");

#if DRIVER_SSD1306_I2C
    ssd1306_init_i2c();
    ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, false);
#else
    ssd1306_init_spi();
    ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, true);
#endif

    HAL_Delay(1000);
    ssd1306_display();
    HAL_Delay(1000*3);

    testdrawline();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
#if 0
        if (timer7_flag) {
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
            printf("dogma...\r\n");
            printf("hello monpetit...\r\n");
            printf("vladimir hamasky...\r\n");
            timer7_flag = false;
        }
#endif
        ssd1306_clear_display();
        // draw a single pixel
        ssd1306_draw_pixel(10, 10, WHITE);
        ssd1306_display();
        HAL_Delay(DELAY_MS);

        ssd1306_draw_circle(SSD1306_LCDWIDTH / 2, SSD1306_LCDHEIGHT / 2, 30, WHITE);
        ssd1306_display();
        HAL_Delay(DELAY_MS);

        testdrawchar();
        HAL_Delay(DELAY_MS);

        ssd1306_clear_display();
        ssd1306_display();
        HAL_Delay(DELAY_MS);

        testdrawline();

        ssd1306_clear_display();
        ssd1306_draw_bitmap(30, 16,  logo16_glcd_bmp, 16, 16, 1);
        ssd1306_display();
        HAL_Delay(DELAY_MS);

        // draw a bitmap icon and 'animate' movement
        testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);
    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void testdrawchar(void)
{
    ssd1306_clear_display();
    ssd1306_set_textsize(1);
    ssd1306_set_textcolor(WHITE);
    ssd1306_set_cursor(0, 0);

    for (uint8_t i = 0; i < 168; i++) {
        if (i == '\n') continue;
        ssd1306_write(i);
        if ((i > 0) && (i % 21 == 0))
            ssd1306_write('\n');
    }
    ssd1306_display();
}


void testdrawline(void)
{
    for (int16_t i = 0; i < ssd1306_width(); i += 4) {
        ssd1306_draw_line(0, 0, i, ssd1306_height() - 1, WHITE);
        ssd1306_display();
    }
    for (int16_t i = 0; i < ssd1306_height(); i += 4) {
        ssd1306_draw_line(0, 0, ssd1306_width() - 1, i, WHITE);
        ssd1306_display();
    }
    HAL_Delay(250);

    ssd1306_clear_display();
    for (int16_t i = 0; i < ssd1306_width(); i += 4) {
        ssd1306_draw_line(0, ssd1306_height() - 1, i, 0, WHITE);
        ssd1306_display();
    }
    for (int16_t i = ssd1306_height() - 1; i >= 0; i -= 4) {
        ssd1306_draw_line(0, ssd1306_height() - 1, ssd1306_width() - 1, i, WHITE);
        ssd1306_display();
    }
    HAL_Delay(250);

    ssd1306_clear_display();
    for (int16_t i = ssd1306_width() - 1; i >= 0; i -= 4) {
        ssd1306_draw_line(ssd1306_width() - 1, ssd1306_height() - 1, i, 0, WHITE);
        ssd1306_display();
    }
    for (int16_t i = ssd1306_height() - 1; i >= 0; i -= 4) {
        ssd1306_draw_line(ssd1306_width() - 1, ssd1306_height() - 1, 0, i, WHITE);
        ssd1306_display();
    }
    HAL_Delay(250);

    ssd1306_clear_display();
    for (int16_t i = 0; i < ssd1306_height(); i += 4) {
        ssd1306_draw_line(ssd1306_width() - 1, 0, 0, i, WHITE);
        ssd1306_display();
    }
    for (int16_t i = 0; i < ssd1306_width(); i += 4) {
        ssd1306_draw_line(ssd1306_width() - 1, 0, i, ssd1306_height() - 1, WHITE);
        ssd1306_display();
    }
    HAL_Delay(250);

    ssd1306_display();
    HAL_Delay(250);
    ssd1306_clear_display();
}


void testdrawbitmap(const uint8_t *bitmap, uint8_t w, uint8_t h)
{
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

    uint8_t icons[NUMFLAKES][3];

    // initialize
    for (uint8_t f = 0; f < NUMFLAKES; f++) {
        icons[f][XPOS] = rand() % ssd1306_width();
        icons[f][YPOS] = 0;
        icons[f][DELTAY] = (rand() % 5) + 1;
    }

    while (1) {
        // draw each icon
        for (uint8_t f = 0; f < NUMFLAKES; f++) {
            ssd1306_draw_bitmap(icons[f][XPOS], icons[f][YPOS], logo16_glcd_bmp, w, h, WHITE);
        }
        ssd1306_display();
        HAL_Delay(200);

        // then erase it + move it
        for (uint8_t f = 0; f < NUMFLAKES; f++) {
            ssd1306_draw_bitmap(icons[f][XPOS], icons[f][YPOS],  logo16_glcd_bmp, w, h, BLACK);
            // move it
            icons[f][YPOS] += icons[f][DELTAY];
            // if its gone, reinit
            if (icons[f][YPOS] > ssd1306_height()) {
                icons[f][XPOS] = rand() % ssd1306_width();
                icons[f][YPOS] = 0;
                icons[f][DELTAY] = (rand() % 5) + 1;
            }
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
