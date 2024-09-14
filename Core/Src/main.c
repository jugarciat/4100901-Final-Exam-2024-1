/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include "ring_buffer.h"
#include "keypad.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define KEYPAD_RB_LEN 4
uint8_t keypad_data = 0xFF;
uint8_t keypad_buffer[KEYPAD_RB_LEN];
ring_buffer_t keypad_rb;

#define USART2_RB_LEN 4
uint8_t usart2_data = 0xFF;
uint8_t usart2_buffer[USART2_RB_LEN];
ring_buffer_t usart2_rb;

#define KEYPAD_MAX_DIGITS 4
uint16_t keypad_value = 0;  // Almacenar el valor actual del keypad
uint8_t keypad_digit_count = 0;  // Número de dígitos recibidos
// Declarar las variables globalmente

uint32_t usart_value = 0;   // Valor recibido por USART2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Función para transmitir datos a través del UART.
 * @param file: Descriptor del archivo (no utilizado).
 * @param ptr: Puntero al buffer de datos a transmitir.
 * @param len: Número de bytes a transmitir.
 * @retval Número de bytes transmitidos.
 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
}


/**
  * @brief  Configura el reloj del sistema para usar el oscilador interno MSI (Multi-Speed Internal)
  *         en lugar del HSI (High-Speed Internal). Esto reduce el consumo de energía al usar un
  *         reloj interno de menor frecuencia. El reloj del sistema se establece en la gama 0 del MSI
  *         (100 kHz).
  *
  *         La función realiza los siguientes pasos:
  *         1. Configura el oscilador MSI en el rango 0 (100 kHz).
  *         2. Establece el MSI como la fuente del reloj del sistema.
  *         3. Desactiva el HSI para reducir el consumo de energía.
  *
  * @retval None
  */
void SystemClock_Decrease(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* Configuración del oscilador MSI en el rango 0 (100 kHz) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    // Configura el oscilador MSI
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        // Error en la inicialización del oscilador MSI
        Error_Handler();
    }

    /* Selecciona el MSI como fuente del reloj del sistema y mantiene los divisores de HCLK, PCLK1 y PCLK2 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;

    // Configura el reloj del sistema para usar el MSI
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        // Error en la configuración del reloj del sistema
        Error_Handler();
    }

    /* Desactiva el HSI para reducir el consumo de energía ya que se usa MSI a partir de este punto */
    __HAL_RCC_HSI_DISABLE();
}





/**
 * @brief Callback que se ejecuta cuando se recibe un dato a través de USART2.
 * @param huart: Handler del UART que generó la interrupción.
 */



/**
  * @brief  Enters a low-power sleep mode to reduce power consumption.
  *         The microcontroller will enter sleep mode for a specified duration,
  *         during which most peripherals will be disabled to save power.
  *         The system will wake up when the user push-button is pressed.
  *
  *         This function uses the System Tick Timer (SysTick) to keep track of
  *         the sleep duration. When the specified awake time has elapsed,
  *         the system will re-enable the necessary peripherals and resume normal operation.
  *
  * @note   The awake time is defined as 5 seconds (5000 milliseconds).
  *         The function disables all AHB and APB peripherals to minimize power consumption.
  *         It also suspends the SysTick timer to prevent unwanted wake-ups and then resumes it
  *         after exiting sleep mode.
  *
  * @retval None
  */
void low_power_sleep_mode(void)
{
    #define AWAKE_TIME (5 * 1000) // 5 segundos (5000 milisegundos)

    static uint32_t sleep_tick = AWAKE_TIME; // Inicializa el tiempo de sueño

    // Verifica si el tiempo actual es menor que el tiempo de sueño restante
    if (sleep_tick > HAL_GetTick()) {
        return; // No es momento de entrar en modo de sueño
    }

    printf("Sleeping\r\n");

    // Actualiza el tiempo de sueño para la próxima vez
    sleep_tick = HAL_GetTick() + AWAKE_TIME;

    // Desactiva los relojes de los periféricos AHB y APB para reducir el consumo de energía
    RCC->AHB1SMENR  = 0x0;
    RCC->AHB2SMENR  = 0x0;
    RCC->AHB3SMENR  = 0x0;

    RCC->APB1SMENR1 = 0x0;
    RCC->APB1SMENR2 = 0x0;
    RCC->APB2SMENR  = 0x0;

    // Suspende el incremento del SysTick para evitar que el modo de sueño sea interrumpido
    HAL_SuspendTick();

    // Entra en modo de sueño. El microcontrolador se despertará cuando se presione el botón del usuario
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    // Reanuda el interruptor del SysTick si fue deshabilitado antes de entrar en modo de sueño
    HAL_ResumeTick();

    printf("Awake\r\n");
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Data received in USART2 */
    if (huart->Instance == USART2) {
        if (usart2_data >= '0' && usart2_data <= '9') {
            ring_buffer_write(&usart2_rb, usart2_data);
            usart_value = usart_value * 10 + (usart2_data - '0');
        } else if (usart2_data == '#') {
            usart_value = 0;  // Reiniciar el valor por USART2
        }
        HAL_UART_Receive_IT(&huart2, &usart2_data, 1);
    }
}

/**
 * @brief Muestra el valor del keypad en la pantalla OLED.
 * @param value: Valor a mostrar.
 */


void display_keypad_value(uint16_t value) {
    // Limpiar la pantalla antes de escribir el nuevo valor
    ssd1306_Fill(Black);

    // Mover el cursor a una posición visible (por ejemplo, en la parte superior de la pantalla)
    ssd1306_SetCursor(0, 0);

    // Crear un buffer para almacenar el texto que se va a mostrar
    char buffer[10];

    // Convertir el valor numérico a una cadena de caracteres
    sprintf(buffer, "Keypad: %d", value);

    // Escribir la cadena en la pantalla OLED
    ssd1306_WriteString(buffer, Font_7x10, White);

    // Actualizar la pantalla para mostrar los cambios
    ssd1306_UpdateScreen();
}

/**
 * @brief Muestra el valor recibido por USART en la pantalla OLED.
 * @param value: Valor a mostrar.
 */
void display_usart_value(uint32_t value) {
    ssd1306_SetCursor(0, 20);  // Ajustar cursor para mostrar en otra línea
    char buffer[10];
    sprintf(buffer, "USART: %d", value);  // Mostrar el valor de USART
    ssd1306_WriteString(buffer, Font_7x10, White);
    ssd1306_UpdateScreen();
}

/**
 * @brief Callback de interrupción del GPIO.
 * @param GPIO_Pin: Pin que generó la interrupción.
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) {
        uint32_t result = keypad_value + usart_value;
        char buffer[50];

        // Mostrar resultado en OLED
        ssd1306_Fill(Black);  // Limpiar pantalla
        ssd1306_SetCursor(0, 0);
        sprintf(buffer, "Sum: %lu", result);
        ssd1306_WriteString(buffer, Font_7x10, White);
        ssd1306_UpdateScreen();

        // Enviar resultado al PC por USART2
        sprintf(buffer, "Result: %lu\r\n", result);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

        // Encender o apagar el LED según si el resultado es par o impar
        if (result % 2 == 0) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Encender LED
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Apagar LED
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(20, 20);
  ssd1306_WriteString("Welcome!", Font_7x10, White);
  ssd1306_UpdateScreen();

  HAL_UART_Receive_IT(&huart2, &usart2_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Starting\r\n");
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 256000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ROW_2_Pin|ROW_4_Pin|ROW_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin ROW_1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|ROW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COL_1_Pin */
  GPIO_InitStruct.Pin = COL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COL_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COL_4_Pin */
  GPIO_InitStruct.Pin = COL_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COL_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COL_2_Pin COL_3_Pin */
  GPIO_InitStruct.Pin = COL_2_Pin|COL_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW_2_Pin ROW_4_Pin ROW_3_Pin */
  GPIO_InitStruct.Pin = ROW_2_Pin|ROW_4_Pin|ROW_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
