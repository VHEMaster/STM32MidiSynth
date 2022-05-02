#include "main.h"
#include "usb_host.h"
#include "usbh_MIDI.h"
#include "arm_math.h"

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
void MX_USB_HOST_Process(void);

#define SAMPLERATE (44100)
#define GET_SIN_FREQ(freq, step) ((float)(freq) / (float)(SAMPLERATE) * 2.0f * (PI) * (step))
#define GET_NOTE_FREQ(note) (440.0f * powf(2.0f, ((float)(note)-69.0f)/12.0f))
#define I2S_BUFFER_SIZE 16

float gFreqs[128];

struct sNote {
    int8_t on;
    int8_t velocity;
    uint32_t step;
    float state;
    float release;
};

struct sNotes {
    struct sNote notes[127];
};

struct sNotes gNotes = {{{0}}};
uint16_t I2S_Buffer[I2S_BUFFER_SIZE];
uint16_t I2S_BufferSingleCh[I2S_BUFFER_SIZE/2];

void I2S_CopyMonoToStereo(uint16_t *bufStereo, uint16_t *bufMono, uint32_t sizeMono)
{
  for(int m = 0, s = 0; m < sizeMono; m++)
  {
    bufStereo[s++] = bufMono[m];
    bufStereo[s++] = bufMono[m];
  }
}

void I2S_HandleBuffer(uint16_t *buffer, uint32_t size)
{
  float sample;
  int16_t *ptr = (int16_t *)buffer;

  for(int j = 0; j < size; j++)
  {
    sample = 0;
    for(int i = 21; i <= 108; i++)
    {
      if(gNotes.notes[i].on)
      {
        gNotes.notes[i].release = ((float)gNotes.notes[i].velocity / 127.0f);
      }
      else
      {
        gNotes.notes[i].release *= 0.9999f;
      }

      if(gNotes.notes[i].release > 0.001f)
      {
        /*
        if(SAMPLERATE / gFreqs[i] > gNotes.notes[i].step % (int)(SAMPLERATE / gFreqs[i]) * 2)
          sample += 1.0f * gNotes.notes[i].release;
        else sample += -1.0f * gNotes.notes[i].release;
        if(SAMPLERATE / gFreqs[i]*2 > gNotes.notes[i].step % (int)(SAMPLERATE / gFreqs[i]*2) * 2)
          sample += 1.0f * gNotes.notes[i].release * 0.7f;
        else sample += -1.0f * gNotes.notes[i].release * 0.7f;
        if(SAMPLERATE / gFreqs[i]*3 > gNotes.notes[i].step % (int)(SAMPLERATE / gFreqs[i]*3) * 2)
          sample += 1.0f * gNotes.notes[i].release * 0.5f;
        else sample += -1.0f * gNotes.notes[i].release * 0.5f;
        */

        sample += arm_sin_f32(GET_SIN_FREQ(gFreqs[i] * 1.0f, gNotes.notes[i].step)) * gNotes.notes[i].release;
        //sample += arm_sin_f32(GET_SIN_FREQ(gFreqs[i] * 2.0f, gNotes.notes[i].step)) * gNotes.notes[i].release;
        //sample += arm_sin_f32(GET_SIN_FREQ(gFreqs[i] * 3.0f, gNotes.notes[i].step)) * gNotes.notes[i].release;
        gNotes.notes[i].step++;
      }
      else
      {
        gNotes.notes[i].step = 0;
      }
    }

    ptr[j] = sample * 32767.0f * 0.05f;

  }
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  I2S_HandleBuffer(&I2S_BufferSingleCh[0], I2S_BUFFER_SIZE / 2 / 2);
  I2S_CopyMonoToStereo(&I2S_Buffer[0], &I2S_BufferSingleCh[0], I2S_BUFFER_SIZE / 2 / 2);
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  I2S_HandleBuffer(&I2S_BufferSingleCh[I2S_BUFFER_SIZE / 2 / 2], I2S_BUFFER_SIZE / 2 / 2);
  I2S_CopyMonoToStereo(&I2S_Buffer[I2S_BUFFER_SIZE / 2], &I2S_BufferSingleCh[I2S_BUFFER_SIZE / 2 / 2], I2S_BUFFER_SIZE / 2 / 2);
}

#define DAC_ADDR 0x94
uint8_t DAC_Read(uint8_t addr)
{
  if(HAL_I2C_Master_Transmit(&hi2c1,DAC_ADDR,&addr,1,1000) != HAL_OK)
    return 0xFF;
  if(HAL_I2C_Master_Receive(&hi2c1,DAC_ADDR+1,&addr,1,1000) != HAL_OK)
    return 0xFE;
  return addr;
}

uint8_t DAC_Write(uint8_t addr, uint8_t data)
{
  uint8_t d[2] = {addr,data};
  if(HAL_I2C_Master_Transmit(&hi2c1,DAC_ADDR,d,2,1000) != HAL_OK)
    return 0xFF;
  return 0;
}

extern ApplicationTypeDef Appli_state;
extern USBH_HandleTypeDef hUsbHostFS;

#define MIDI_RX_BUFFER_SIZE 64
uint8_t MIDI_RX_Buffer[MIDI_RX_BUFFER_SIZE];

int main(void)
{
  HAL_Init();

  SCB_EnableICache();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_USB_HOST_Init();


  HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(I2S2_NRST_GPIO_Port, I2S2_NRST_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  while((DAC_Read(0x01)&0xF8) != 0xE0)
  {
    HAL_Delay(100);
  }

  HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_SET);

  DAC_Write(0x01,0x01);

  DAC_Write(0x05,0xA1);
  DAC_Write(0x06,0x04);
  DAC_Write(0x0F,0x06);
  DAC_Write(0x24,0x00);
  DAC_Write(0x25,0x00);
  DAC_Write(0x0A,0x00);
  DAC_Write(0x0E,0x04);
  DAC_Write(0x27,0x00);
  DAC_Write(0x1F,0x88);

  //VOLUME
  DAC_Write(0x1A,0);
  DAC_Write(0x1B,0);

  DAC_Write(0x22,-80);
  DAC_Write(0x23,-80);

  DAC_Write(0x04,0xAA); //Unmute
  //DAC_Write(0x04,0xFF); //Mute

  DAC_Write(0x02,0x9E); //PowerUP
  //DAC_Write(0x02,0x01); //PowerDown


  //DAC_Write(0x04,0xFF); //Mute

  DAC_Write(0x22,0);
  DAC_Write(0x23,0);


  DAC_Write(0x22,-10);
  DAC_Write(0x23,-10);

  for(int i = 0; i < 128; i++)
  {
    gFreqs[i] = GET_NOTE_FREQ(i);
  }

  HAL_I2S_Transmit_DMA(&hi2s2, I2S_Buffer, I2S_BUFFER_SIZE);

  ApplicationTypeDef state_old = APPLICATION_IDLE;

  while (1)
  {
    MX_USB_HOST_Process();

    if(state_old != Appli_state)
    {
      state_old = Appli_state;
      if(Appli_state == APPLICATION_READY)
      {
        USBH_MIDI_Receive(&hUsbHostFS, MIDI_RX_Buffer, MIDI_RX_BUFFER_SIZE);
      }
      else if (Appli_state == APPLICATION_DISCONNECT)
      {
        Appli_state = APPLICATION_IDLE;
        USBH_MIDI_Stop(&hUsbHostFS);
      }
    }

  }
}

void USBH_MIDI_ReceiveCallback(USBH_HandleTypeDef *phost)
{
  uint16_t numberOfPackets;

  numberOfPackets = USBH_MIDI_GetLastReceivedDataSize(&hUsbHostFS) / 4; //each USB midi package is 4 bytes long
  uint8_t *ptr = MIDI_RX_Buffer;

  uint8_t cin;
  uint8_t event[3];

  int8_t note,velocity;


  for(int i = 0; i < numberOfPackets; i++)
  {
    cin = *ptr++;
    event[0] = *ptr++;
    event[1] = *ptr++;
    event[2] = *ptr++;


    if((cin & 0xF) == 0x8 || (cin & 0xF) == 0x9) //Note Off or Note On
    {
      note = event[1];
      velocity = event[2];

      gNotes.notes[note].velocity = velocity;
      if((cin & 0xF) == 0x9) //Note On
      {
        gNotes.notes[note].on = 1;
        gNotes.notes[note].step = 0;
      }
      else
      {
        gNotes.notes[note].on = 0;
      }
    }

  }

  USBH_MIDI_Receive(&hUsbHostFS, MIDI_RX_Buffer, MIDI_RX_BUFFER_SIZE); // start a new reception
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  hi2c1.Init.Timing = 0x20404768;
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
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2S2_NRST_GPIO_Port, I2S2_NRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_D3_Pin|LED_D4_Pin|LED_D5_Pin|LED_D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUT_G6_Pin BUT_G7_Pin */
  GPIO_InitStruct.Pin = BUT_G6_Pin|BUT_G7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S2_NRST_Pin */
  GPIO_InitStruct.Pin = I2S2_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2S2_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D3_Pin LED_D4_Pin LED_D5_Pin LED_D6_Pin */
  GPIO_InitStruct.Pin = LED_D3_Pin|LED_D4_Pin|LED_D5_Pin|LED_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

