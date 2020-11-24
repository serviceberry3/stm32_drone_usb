/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * sensors_bmi270_spi_bmp388.c: IMU sensor accusation for the bosch sensors
 */

#define DEBUG_MODULE "IMU"

#include <math.h>
#include <string.h>

#include "stm32fxxx.h"

#include "sensors_bmi270_spi_bmp388.h"
#include "imu.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "imu.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "i2cdev.h"
#include "bmi270.h"
#include "bmi2.h"
#include "bmp3.h"
#include "bstdr_types.h"
#include "static_mem.h"

/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
// BMI270 only have one CS pin
// #define BMI270_ACC_GPIO_CS             GPIO_Pin_1
// #define BMI270_ACC_GPIO_CS_PORT        GPIOB
// #define BMI270_ACC_GPIO_CS_PERIF       RCC_AHB1Periph_GPIOB

#define BMI270_GPIO_CS                 GPIO_Pin_0
#define BMI270_GPIO_CS_PORT            GPIOB
#define BMI270_GPIO_CS_PERIF           RCC_AHB1Periph_GPIOB

#define BMI270_SPI                     SPI2
#define BMI270_SPI_AF                  GPIO_AF_SPI2
#define BMI270_SPI_CLK                 RCC_APB1Periph_SPI2
#define BMI270_GPIO_SPI_PORT           GPIOB
#define BMI270_GPIO_SPI_CLK            RCC_AHB1Periph_GPIOB
#define BMI270_GPIO_SPI_SCK            GPIO_Pin_13
#define BMI270_GPIO_SPI_SCK_SRC        GPIO_PinSource13
#define BMI270_GPIO_SPI_MISO           GPIO_Pin_14
#define BMI270_GPIO_SPI_MISO_SRC       GPIO_PinSource14
#define BMI270_GPIO_SPI_MOSI           GPIO_Pin_15
#define BMI270_GPIO_SPI_MOSI_SRC       GPIO_PinSource15

#define BMI270_SPI_DMA_IRQ_PRIO        NVIC_HIGH_PRI
#define BMI270_SPI_DMA                 DMA1
#define BMI270_SPI_DMA_CLK             RCC_AHB1Periph_DMA1
#define BMI270_SPI_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

#define BMI270_SPI_RX_DMA_STREAM       DMA1_Stream3
#define BMI270_SPI_RX_DMA_IRQ          DMA1_Stream3_IRQn
#define BMI270_SPI_RX_DMA_IRQHandler   DMA1_Stream3_IRQHandler
#define BMI270_SPI_RX_DMA_CHANNEL      DMA_Channel_0
#define BMI270_SPI_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF3

#define BMI270_SPI_TX_DMA_STREAM       DMA1_Stream4
#define BMI270_SPI_TX_DMA_IRQ          DMA1_Stream4_IRQn
#define BMI270_SPI_TX_DMA_IRQHandler   DMA1_Stream4_IRQHandler
#define BMI270_SPI_TX_DMA_CHANNEL      DMA_Channel_0
#define BMI270_SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF4

#define DUMMY_BYTE    0x00

#define SENSORS_READ_RATE_HZ            1000
#define SENSORS_STARTUP_TIME_MS         1000
#define SENSORS_READ_BARO_HZ            50
#define SENSORS_READ_MAG_HZ             20
#define SENSORS_DELAY_BARO              (SENSORS_READ_RATE_HZ / SENSORS_READ_BARO_HZ)
#define SENSORS_DELAY_MAG               (SENSORS_READ_RATE_HZ / SENSORS_READ_MAG_HZ)

#define SENSORS_BMI270_DEG_PER_LSB_CFG  (2.0f *2000.0f) / 65536.0f

#define SENSORS_BMI270_ACCEL_CFG        16
// #define SENSORS_BMI270_ACCEL_FS_CFG     BMI270_ACCEL_RANGE_24G
#define SENSORS_BMI270_G_PER_LSB_CFG    (2.0f * (float) SENSORS_BMI270_ACCEL_CFG) / 65536.0f
#define SENSORS_BMI270_1G_IN_LSB        (65536 / SENSORS_BMI270_ACCEL_CFG / 2)

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT   M2T(1000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX          5.0f      // Max degrees off

#define GYRO_NBR_OF_AXES                3
#define GYRO_MIN_BIAS_TIMEOUT_MS        M2T(1*1000)

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES  512

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE              10000
#define GYRO_VARIANCE_THRESHOLD_X       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z       (GYRO_VARIANCE_BASE)

#define SENSORS_ACC_SCALE_SAMPLES  200

/* Usefull macro */
#define BMI_EN_CS() GPIO_ResetBits(BMI270_GPIO_CS_PORT, BMI270_GPIO_CS)
#define BMI_DIS_CS() GPIO_SetBits(BMI270_GPIO_CS_PORT, BMI270_GPIO_CS)

/* Defines and buffers for full duplex SPI DMA transactions */
/* The buffers must not be placed in CCM */
#define SPI_MAX_DMA_TRANSACTION_SIZE    30
static uint8_t spiTxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static uint8_t spiRxBuffer[SPI_MAX_DMA_TRANSACTION_SIZE + 1];
static xSemaphoreHandle spiTxDMAComplete;
static StaticSemaphore_t spiTxDMACompleteBuffer;
static xSemaphoreHandle spiRxDMAComplete;
static StaticSemaphore_t spiRxDMACompleteBuffer;

typedef struct {
  Axis3f     bias;
  Axis3f     variance;
  Axis3f     mean;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

/* initialize necessary variables */
static struct bmi2_dev   bmi270Dev;
static struct bmp3_dev   bmp388Dev;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle sensorsDataReady;
static StaticSemaphore_t sensorsDataReadyBuffer;
static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;

static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined (GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;
static bool accScaleFound = false;
static uint32_t accScaleSumCount = 0;


// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);

static bool isBarometerPresent = false;
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO;

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz,  Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

/***********************
 * SPI private methods *
 ***********************/
static char spiSendByte(char byte) {
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(BMI270_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI peripheral */
  SPI_I2S_SendData(BMI270_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(BMI270_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(BMI270_SPI);
}

static char spiReceiveByte() {
  return spiSendByte(DUMMY_BYTE);
}

static void spiDMATransaction(uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

  ASSERT(len < SPI_MAX_DMA_TRANSACTION_SIZE);

  // Disable peripheral before setting up for duplex DMA
  SPI_Cmd(BMI270_SPI, DISABLE);

  // DMA already configured, just need to set memory addresses and read command byte
  spiTxBuffer[0] = reg_addr;
  BMI270_SPI_TX_DMA_STREAM->M0AR = (uint32_t) &spiTxBuffer[0];
  BMI270_SPI_TX_DMA_STREAM->NDTR = len + 1;

  BMI270_SPI_RX_DMA_STREAM->M0AR = (uint32_t) &spiRxBuffer[0];
  BMI270_SPI_RX_DMA_STREAM->NDTR = len + 1;

  // Enable SPI DMA Interrupts
  DMA_ITConfig(BMI270_SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(BMI270_SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Clear DMA Flags
  DMA_ClearFlag(BMI270_SPI_TX_DMA_STREAM, DMA_FLAG_FEIF4|DMA_FLAG_DMEIF4|
                DMA_FLAG_TEIF4|DMA_FLAG_HTIF4|DMA_FLAG_TCIF4);
  DMA_ClearFlag(BMI270_SPI_RX_DMA_STREAM, DMA_FLAG_FEIF3|DMA_FLAG_DMEIF3|
                DMA_FLAG_TEIF3|DMA_FLAG_HTIF3|DMA_FLAG_TCIF3);

  // Enable DMA Streams
  DMA_Cmd(BMI270_SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(BMI270_SPI_RX_DMA_STREAM,ENABLE);

  // Enable SPI DMA requests
  SPI_I2S_DMACmd(BMI270_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(BMI270_SPI, SPI_I2S_DMAReq_Rx, ENABLE);

  // Enable peripheral to begin the transaction
  SPI_Cmd(BMI270_SPI, ENABLE);

  // Wait for completion
  // TODO: Better error handling rather than passing up invalid data
  xSemaphoreTake(spiTxDMAComplete, portMAX_DELAY);
  xSemaphoreTake(spiRxDMAComplete, portMAX_DELAY);

  // Copy the data (discarding the dummy byte) into the buffer
  // TODO: Avoid this memcpy either by figuring out how to configure the STM SPI to discard the byte or handle it higher up
  memcpy(reg_data, &spiRxBuffer[1], len);
}

// Communication routines
static BMI2_INTF_RETURN_TYPE bmi_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
  BMI_EN_CS();

  spiSendByte(reg_addr);
  for (int i = 0; i < len; i++) {
    spiSendByte(reg_data[i]);
  }

  BMI_DIS_CS();
  return BMI2_INTF_RET_SUCCESS;
}

static BMI2_INTF_RETURN_TYPE bmi_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void *intf_ptr) {
  BMI_EN_CS();

  if (len <= 1 || len > SPI_MAX_DMA_TRANSACTION_SIZE) {
    // DEBUG_PRINT("spi_read: %x, %ld\n", reg_addr, len);
    spiSendByte(reg_addr);
    for (int i = 0; i < len; i++)
      reg_data[i] = spiReceiveByte();

  } else {

    spiDMATransaction(reg_addr, reg_data, len);
  }

  BMI_DIS_CS();
  
  return BMI2_INTF_RET_SUCCESS;
}

// static bstdr_ret_t spi_burst_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
//   /**< Burst read code comes here */
//   if (dev_id == BMI270_ACCEL_I2C_ADDR_PRIMARY) {
    // ACC_EN_CS();
//   } else {
//     GYR_EN_CS();
//   }

//   if (len <= 1 || len > SPI_MAX_DMA_TRANSACTION_SIZE) {
//     spiSendByte(reg_addr);
//     for (int i = 0; i < len; i++) {
//       reg_data[i] = spiReceiveByte();
//     }
//   } else {
//     spiDMATransaction(reg_addr, reg_data, len);
//   }

//   if (dev_id == BMI270_ACCEL_I2C_ADDR_PRIMARY) {
//     ACC_DIS_CS();
//   } else {
//     GYR_DIS_CS();
//   }

//   return BSTDR_OK;
// }

// static bstdr_ret_t spi_burst_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
//   if (dev_id == BMI270_ACCEL_I2C_ADDR_PRIMARY) {
//     ACC_EN_CS();
//   } else {
//     GYR_EN_CS();
//   }
//   spiSendByte(reg_addr);
//   for (int i = 0; i < len; i++) {
//     spiSendByte(reg_data[i]);
//   }

//   if (dev_id == BMI270_ACCEL_I2C_ADDR_PRIMARY) {
//     ACC_DIS_CS();
//   } else {
//     GYR_DIS_CS();
//   }

//   return BSTDR_OK;
// }

/***********************
 * I2C private methods *
 ***********************/
static bstdr_ret_t i2c_burst_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  /**< Burst read code comes here */
  if (i2cdevReadReg8(I2C3_DEV, dev_id, reg_addr, (uint16_t) len, reg_data)) {
    return BSTDR_OK;
  } else {
    return BSTDR_E_CON_ERROR;
  }
}

static bstdr_ret_t i2c_burst_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  /**< Burst write code comes here */
  if (i2cdevWriteReg8(I2C3_DEV, dev_id,reg_addr,(uint16_t) len, reg_data)) {
    return BSTDR_OK;
  } else {
    return BSTDR_E_CON_ERROR;
  }
}


static void bmi270_us_delay(uint32_t period, void *intf_ptr) {
  /**< Delay code comes */
  uint32_t period_ms = period / 1000.0;
  if (period < 1000) {
    // TIM_SetCounter(TIM13, 0);
    // while (TIM_GetCounter(TIM13) < period);
    // for (int i = 0; i < period; i++) {}
    vTaskDelay(M2T(1));
  } else
    vTaskDelay(M2T(period_ms));
}

// static void delay_us(uint32_t us, void *intf_ptr) {
//   TIM_SetCounter(TIM13, 0);
//   while (TIM_GetCounter(TIM13) < us);
// }

static void bmi388_ms_delay(uint32_t period) {
  /**< Delay code comes */
  vTaskDelay(M2T(period));
}

static void spiConfigure(void) {
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_Cmd(BMI270_SPI, DISABLE);
  SPI_I2S_DeInit(BMI270_SPI);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; // ~10.5 MHz
  SPI_Init(BMI270_SPI, &SPI_InitStructure);
  /* Enable the SPI  */
  SPI_Cmd(BMI270_SPI, ENABLE);
}

/* Initialisation */
static void spiInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  if (isInit)
    return;

  /* Enable SPI and GPIO clocks */
  RCC_AHB1PeriphClockCmd(BMI270_GPIO_SPI_CLK | BMI270_GPIO_CS_PERIF, ENABLE);
  /* Enable SPI and GPIO clocks */
  RCC_APB1PeriphClockCmd(BMI270_SPI_CLK, ENABLE);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = BMI270_GPIO_SPI_SCK |  BMI270_GPIO_SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BMI270_GPIO_SPI_PORT, &GPIO_InitStructure);

  //* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = BMI270_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(BMI270_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  // GPIO_InitStructure.GPIO_Pin = BMI270_ACC_GPIO_CS;
  // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  // GPIO_Init(BMI270_ACC_GPIO_CS_PORT, &GPIO_InitStructure);
  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = BMI270_GPIO_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(BMI270_GPIO_CS_PORT, &GPIO_InitStructure);


  /*!< Connect SPI pins to AF5 */
  GPIO_PinAFConfig(BMI270_GPIO_SPI_PORT, BMI270_GPIO_SPI_SCK_SRC, BMI270_SPI_AF);
  GPIO_PinAFConfig(BMI270_GPIO_SPI_PORT, BMI270_GPIO_SPI_MISO_SRC, BMI270_SPI_AF);
  GPIO_PinAFConfig(BMI270_GPIO_SPI_PORT, BMI270_GPIO_SPI_MOSI_SRC, BMI270_SPI_AF);

  /* disable the chip select */
  BMI_DIS_CS();

  spiConfigure();
}

static void spiDMAInit(void) {
  DMA_InitTypeDef  DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /*!< Enable DMA Clocks */
  BMI270_SPI_DMA_CLK_INIT(BMI270_SPI_DMA_CLK, ENABLE);

  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(BMI270_SPI->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_BufferSize = 0; // set later
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // set later

  // Configure TX DMA
  DMA_InitStructure.DMA_Channel = BMI270_SPI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Cmd(BMI270_SPI_TX_DMA_STREAM,DISABLE);
  DMA_Init(BMI270_SPI_TX_DMA_STREAM, &DMA_InitStructure);

  // Configure RX DMA
  DMA_InitStructure.DMA_Channel = BMI270_SPI_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_Cmd(BMI270_SPI_RX_DMA_STREAM, DISABLE);
  DMA_Init(BMI270_SPI_RX_DMA_STREAM, &DMA_InitStructure);

  // Configure interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_InitStructure.NVIC_IRQChannel = BMI270_SPI_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = BMI270_SPI_RX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);

  spiTxDMAComplete = xSemaphoreCreateBinaryStatic(&spiTxDMACompleteBuffer);
  spiRxDMAComplete = xSemaphoreCreateBinaryStatic(&spiRxDMACompleteBuffer);
}

static uint8_t sensorsGyroGet(Axis3i16* dataOut) {
  int8_t rslt;
  struct bmi2_sensor_data sensor_data = { 0 };
  sensor_data.type = BMI2_GYRO;
  rslt = bmi2_get_sensor_data(&sensor_data, 1, &bmi270Dev);
  dataOut->x = sensor_data.sens_data.gyr.x;
  dataOut->y = sensor_data.sens_data.gyr.y;
  dataOut->z = sensor_data.sens_data.gyr.z;
  return rslt;
}

static uint8_t sensorsAccelGet(Axis3i16* dataOut) {
  int8_t rslt;
  struct bmi2_sensor_data sensor_data = { 0 };
  sensor_data.type = BMI2_ACCEL;
  rslt = bmi2_get_sensor_data(&sensor_data, 1, &bmi270Dev);
  dataOut->x = sensor_data.sens_data.acc.x;
  dataOut->y = sensor_data.sens_data.acc.y; 
  dataOut->z = sensor_data.sens_data.acc.z;
  return rslt;
}

static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature) {
  baroScaled->pressure = pressure * 0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f)
      - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

bool sensorsBmi270SpiBmp388ReadGyro(Axis3f *gyro) {
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsBmi270SpiBmp388ReadAcc(Axis3f *acc) {
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsBmi270SpiBmp388ReadMag(Axis3f *mag) {
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsBmi270SpiBmp388ReadBaro(baro_t *baro) {
  return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsBmi270SpiBmp388Acquire(sensorData_t *sensors, const uint32_t tick) {
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi270SpiBmp388AreCalibrated() {
  return gyroBiasFound;
}

static void sensorsTask(void *param) {
  systemWaitStart();
  Axis3f accScaled;
  /* wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  // vTaskDelayUntil(&lastWakeTime, M2T(1500));
  DEBUG_PRINT("sensor task begins\n");
  while (1) {

    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
      sensorData.interruptTimestamp = imuIntTimestamp;
      /* get data from chosen sensors */
      sensorsGyroGet(&gyroRaw);
      sensorsAccelGet(&accelRaw);

      /* calibrate if necessary */
#ifdef GYRO_BIAS_LIGHT_WEIGHT
      gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
      gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif
      if (gyroBiasFound) {
         processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
      }
      /* Gyro */
      sensorData.gyro.x =  (gyroRaw.x - gyroBias.x) * SENSORS_BMI270_DEG_PER_LSB_CFG;
      sensorData.gyro.y =  (gyroRaw.y - gyroBias.y) * SENSORS_BMI270_DEG_PER_LSB_CFG;
      sensorData.gyro.z =  (gyroRaw.z - gyroBias.z) * SENSORS_BMI270_DEG_PER_LSB_CFG;
      applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

      /* Acelerometer */
      accScaled.x = accelRaw.x * SENSORS_BMI270_G_PER_LSB_CFG / accScale;
      accScaled.y = accelRaw.y * SENSORS_BMI270_G_PER_LSB_CFG / accScale;
      accScaled.z = accelRaw.z * SENSORS_BMI270_G_PER_LSB_CFG / accScale;
      sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
      applyAxis3fLpf((lpf2pData*)(&accLpf), &sensorData.acc);

    }

    if (isBarometerPresent) {
      static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
      if (--baroMeasDelay == 0) {
        uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
        struct bmp3_data data;
        baro_t* baro388 = &sensorData.baro;
        /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);
        sensorsScaleBaro(baro388, data.pressure, data.temperature);
        baroMeasDelay = baroMeasDelayMin;
      }
    }

    // overwrite to queue
    xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
    xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
    if (isBarometerPresent)
      xQueueOverwrite(barometerDataQueue, &sensorData.baro);

    xSemaphoreGive(dataReady);
  }
}

void sensorsBmi270SpiBmp388WaitDataReady(void) {
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

static void sensorsDeviceInit(void) {
  if (isInit)
    return;

  bstdr_ret_t rslt;
  isBarometerPresent = false;

  // Wait for sensors to startup
  vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));
  

  // init TIM13
  // TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  // TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  // RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM13, ENABLE);
  // TIM_TimeBaseStructure.TIM_Period = 0xffff - 1;
  // TIM_TimeBaseStructure.TIM_Prescaler = 83;
  // TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  // TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  // TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  // // if (0)
  // TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
  // TIM_Cmd(TIM13, ENABLE);


  /* BMI270 */
  DEBUG_PRINT("Begin BMI270 init\n");
  bmi270Dev.intf = BMI2_SPI_INTF;
  bmi270Dev.read = bmi_spi_read;
  bmi270Dev.write = bmi_spi_write;
  bmi270Dev.read_write_len = 30;
  bmi270Dev.config_file_ptr = NULL;
  bmi270Dev.delay_us = bmi270_us_delay;
  bmi270Dev.aps_status = BMI2_DISABLE;
  rslt = bmi270_init(&bmi270Dev);

  if (rslt != BMI2_OK) {
    DEBUG_PRINT("BMI270 init [FAIL: %d]\n", rslt);
    isInit = false;
  }

  uint8_t chip_id;
  rslt = bmi2_get_regs(BMI2_CHIP_ID_ADDR, &chip_id, 1, &bmi270Dev);

  if (chip_id == BMI270_CHIP_ID && rslt == BMI2_OK) {
    DEBUG_PRINT("BMI270 SPI connection with id = 0x%x [OK]\n", chip_id);

    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, &bmi270Dev);
    if (rslt != BMI2_OK) {
      DEBUG_PRINT("BMI270 enable [FAIL]\n");
      isInit = false;
    }

    struct bmi2_sens_config sens_cfg[2];
    sens_cfg[0].type = BMI2_ACCEL;
    sens_cfg[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ;
    sens_cfg[0].cfg.acc.bwp =BMI2_ACC_NORMAL_AVG4;
    sens_cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[0].cfg.acc.range = BMI2_ACC_RANGE_16G;

    sens_cfg[1].type = BMI2_GYRO;
    sens_cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
    sens_cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    sens_cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sens_cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sens_cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

    rslt = bmi2_set_sensor_config(sens_cfg, 2, &bmi270Dev);
    
    if (rslt != BMI2_OK) {
      DEBUG_PRINT("BMI270 config [FAIL]\n");
      isInit = false;
    }
    
    // enable interrupt over INT2 --- PC14
    struct bmi2_int_pin_config data_int_cfg;
    data_int_cfg.pin_type = BMI2_INT2;
    data_int_cfg.int_latch = BMI2_INT_NON_LATCH;
    data_int_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; // Output enabled
    data_int_cfg.pin_cfg[0].od = BMI2_INT_PUSH_PULL;            // OpenDrain disabled
    data_int_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;         // Signal High Active
    data_int_cfg.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;  // Input Disabled
    data_int_cfg.pin_cfg[1].output_en = BMI2_INT_OUTPUT_ENABLE; // Output enabled
    data_int_cfg.pin_cfg[1].od = BMI2_INT_PUSH_PULL;            // OpenDrain disabled
    data_int_cfg.pin_cfg[1].lvl = BMI2_INT_ACTIVE_HIGH;         // Signal High Active
    data_int_cfg.pin_cfg[1].input_en = BMI2_INT_INPUT_DISABLE;  // Input Disabled
    rslt = bmi2_set_int_pin_config(&data_int_cfg, &bmi270Dev);
    if (rslt != BMI2_OK) {
      DEBUG_PRINT("BMI270 set int [FAIL]\n");
      isInit = false;
    }

    bmi270Dev.delay_us(50000, NULL);
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, &bmi270Dev);
    if (rslt != BMI2_OK) {
      DEBUG_PRINT("BMI270 map int [FAIL]\n");
      isInit = false;
    }
    bmi270Dev.delay_us(100000, NULL);
    rslt = sensorsAccelGet(&accelRaw);
    if (rslt != BMI2_OK) {
      DEBUG_PRINT("BMI270 get accel data [FAIL]\n");
      isInit = false;
    } else {
      DEBUG_PRINT("Read accel: %d %d %d\n", accelRaw.x, accelRaw.y, accelRaw.z);
    }

    DEBUG_PRINT("BMI270 int finish [OK]\n");
  } else {
#ifndef SENSORS_IGNORE_IMU_FAIL
    DEBUG_PRINT("BMI270 get id [FAIL]\n");
    isInit = false;
#endif
  }

  /* BMI270 GYRO */
//   rslt = bmi270_gyro_init(&bmi270Dev); // initialize the device
//   if (rslt == BSTDR_OK) {
    // struct bmi088_int_cfg intConfig;

//     DEBUG_PRINT("BMI270 Gyro SPI connection [OK].\n");
//     /* set power mode of gyro */
//     bmi270Dev.gyro_cfg.power = BMI270_GYRO_PM_NORMAL;
//     rslt |= bmi270_set_gyro_power_mode(&bmi270Dev);
//     /* set bandwidth and range of gyro */
    // bmi270Dev.gyro_cfg.bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
    // bmi270Dev.gyro_cfg.range = SENSORS_BMI088_GYRO_FS_CFG;
//     bmi270Dev.gyro_cfg.odr = BMI270_GYRO_BW_116_ODR_1000_HZ;
//     rslt |= bmi270_set_gyro_meas_conf(&bmi270Dev);

//     intConfig.gyro_int_channel = BMI270_INT_CHANNEL_3;
    // intConfig.gyro_int_type = BMI088_GYRO_DATA_RDY_INT;
//     intConfig.gyro_int_pin_3_cfg.enable_int_pin = 1;
//     intConfig.gyro_int_pin_3_cfg.lvl = 1;
//     intConfig.gyro_int_pin_3_cfg.output_mode = 0;
//     /* Setting the interrupt configuration */
//     rslt = bmi270_set_gyro_int_config(&intConfig, &bmi270Dev);

//     bmi270Dev.delay_ms(50);
//     struct bmi2_sensor_data gyr;
//     rslt |= bmi270_get_gyro_data(&gyr, &bmi270Dev);

//   } else {
// #ifndef SENSORS_IGNORE_IMU_FAIL
//     DEBUG_PRINT("BMI270 Gyro SPI connection [FAIL]\n");
//     isInit = false;
// #endif
//   }

//   /* BMI270 ACCEL */
//   rslt |= bmi270_accel_switch_control(&bmi270Dev, BMI270_ACCEL_POWER_ENABLE);
//   bmi270Dev.delay_ms(5);

//   rslt = bmi270_accel_init(&bmi270Dev); // initialize the device
//   if (rslt == BSTDR_OK) {
//     DEBUG_PRINT("BMI270 Accel SPI connection [OK]\n");
//     /* set power mode of accel */
//     bmi270Dev.accel_cfg.power = BMI270_ACCEL_PM_ACTIVE;
//     rslt |= bmi270_set_accel_power_mode(&bmi270Dev);
//     bmi270Dev.delay_ms(10);

//     /* set bandwidth and range of accel */
//     bmi270Dev.accel_cfg.bw = BMI270_ACCEL_BW_OSR4;
    // bmi270Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
//     bmi270Dev.accel_cfg.odr = BMI270_ACCEL_ODR_1600_HZ;
//     rslt |= bmi270_set_accel_meas_conf(&bmi270Dev);

//     struct bmi2_sensor_data acc;
//     rslt |= bmi270_get_accel_data(&acc, &bmi270Dev);
//   } else {
// #ifndef SENSORS_IGNORE_IMU_FAIL
//     DEBUG_PRINT("BMI270 Accel SPI connection [FAIL]\n");
//     isInit = false;
// #endif
//   }

  /* BMP388 */

  bmp388Dev.dev_id = BMP3_I2C_ADDR_SEC;
  bmp388Dev.intf = BMP3_I2C_INTF;
  bmp388Dev.read = i2c_burst_read;
  bmp388Dev.write = i2c_burst_write;
  bmp388Dev.delay_ms = bmi388_ms_delay;

  int i = 3;
  do {
    bmp388Dev.delay_ms(1);
    // For some reason it often doesn't work first time
    rslt = bmp3_init(&bmp388Dev);
  } while (rslt != BMP3_OK && i-- > 0);

  if (rslt == BMP3_OK) {
    isBarometerPresent = true;
    DEBUG_PRINT("BMP388 I2C connection [OK]\n");
    // Used to select the settings user needs to change
    uint16_t settings_sel;
    // Select the pressure and temperature sensor to be enabled
    bmp388Dev.settings.press_en = BMP3_ENABLE;
    bmp388Dev.settings.temp_en = BMP3_ENABLE;
    // Select the output data rate and oversampling settings for pressure and temperature
    bmp388Dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    bmp388Dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    bmp388Dev.settings.odr_filter.odr = BMP3_ODR_50_HZ;
    bmp388Dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    // Assign the settings which needs to be set in the sensor
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL | BMP3_IIR_FILTER_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, &bmp388Dev);

    // Set the power mode to normal mode
    bmp388Dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(&bmp388Dev);


    bmp388Dev.delay_ms(20); // wait before first read out
    // read out data
    // Variable used to select the sensor component
    uint8_t sensor_comp;
    // Variable used to store the compensated data
    struct bmp3_data data;

    // Sensor component selection
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    // Temperature and Pressure data are read and stored in the bmp3_data instance
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);

    // Print the temperature and pressure data
//    DEBUG_PRINT("BMP388 T:%0.2f  P:%0.2f\n",data.temperature, data.pressure/100.0f);
    baroMeasDelayMin = SENSORS_DELAY_BARO;
  } else {
#ifndef SENSORS_IGNORE_BAROMETER_FAIL
    DEBUG_PRINT("BMP388 I2C connection [FAIL]\n");
    isInit = false;
    return;
#endif
  }

  // Init second order filer for accelerometer and gyro
  for (uint8_t i = 0; i < 3; i++) {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
  }

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI / 180);

  isInit = true;
}

static void sensorsTaskInit(void) {
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);
  // return;
  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

static void sensorsInterruptInit(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  sensorsDataReady = xSemaphoreCreateBinaryStatic(&sensorsDataReadyBuffer);
  dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

  // Enable the interrupt on PC14
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);

  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  portDISABLE_INTERRUPTS();
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line14);
  portENABLE_INTERRUPTS();
}

void sensorsBmi270SpiBmp388Init(void) {
  if (isInit) {
    return;
  }

  i2cdevInit(I2C3_DEV);
  spiInit();
  spiDMAInit();

  sensorsBiasObjInit(&gyroBiasRunning);
  
  sensorsDeviceInit();
  sensorsInterruptInit();
  sensorsTaskInit();
}

static bool accelSelftest() {
  bool testStatus = true;
  int i = 3;
  uint16_t readResult = BMI2_OK;
  do {
    readResult = sensorsAccelGet(&accelRaw);
  } while (readResult != BMI2_OK && i-- > 0);

  if ((readResult != BMI2_OK) || (accelRaw.x == 0 && accelRaw.y == 0 && accelRaw.z == 0)) {
    DEBUG_PRINT("BMI270 accel returning x = 0 y = 0 z = 0 [FAILED: %d]\n", readResult);
    testStatus = false;
  } else {
    DEBUG_PRINT("BMI270 accel self-test [OK]\n");
  }
  // Do not perform accel_self_test, otherwise the BMI270 won't fire the interrupt
  // int8_t accelResult = 0;
  // accelResult = bmi2_perform_accel_self_test(&bmi270Dev);
  // if (accelResult == BMI2_OK) {
  //   DEBUG_PRINT("BMI270 accel self-test [OK]\n");
  // } else {
  //   DEBUG_PRINT("BMI270 accel self-test [FAILED]\n");
  //   testStatus = false;
  // }

  return testStatus;
}


bool sensorsBmi270SpiBmp388Test(void) {
  bool testStatus = true;

  if (!isInit) {
    DEBUG_PRINT("Uninitialized\n");
    testStatus = false;
  }

  if (!accelSelftest()) {
    testStatus = false;
  }

  return testStatus;
}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az) {
  if (!accScaleFound) {
    accScaleSum += sqrtf(powf(ax * SENSORS_BMI270_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_BMI270_G_PER_LSB_CFG, 2) + powf(az * SENSORS_BMI270_G_PER_LSB_CFG, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accScaleFound = true;
    }
  }

  return accScaleFound;
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT

#define SENSORS_BIAS_SAMPLES       1000
/**
 * Calculates the bias out of the first SENSORS_BIAS_SAMPLES gathered. Requires no buffer
 * but needs platform to be stable during startup.
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut) {
  static uint32_t gyroBiasSampleCount = 0;
  static bool gyroBiasNoBuffFound = false;
  static Axis3i64 gyroBiasSampleSum;
  static Axis3i64 gyroBiasSampleSumSquares;

  if (!gyroBiasNoBuffFound) {
    // If the gyro has not yet been calibrated:
    // Add the current sample to the running mean and variance
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
    gyroBiasSampleSumSquares.y += gy * gy;
    gyroBiasSampleSumSquares.z += gz * gz;
#endif
    gyroBiasSampleCount += 1;

    // If we then have enough samples, calculate the mean and standard deviation
    if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES) {
      gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
      gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
      gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
      gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
      gyroBiasNoBuffFound = true;
    }
  }

  return gyroBiasNoBuffFound;
}
#else
/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut) {
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

  if (!gyroBiasRunning.isBiasValueFound) {
    sensorsFindBiasValue(&gyroBiasRunning);
    if (gyroBiasRunning.isBiasValueFound) {
      soundSetEffect(SND_CALIB);
      ledseqRun(&seq_calibrated);
    }
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;

  return gyroBiasRunning.isBiasValueFound;
}
#endif

static void sensorsBiasObjInit(BiasObj* bias) {
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut) {
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = { 0 };
  int64_t sumSq[GYRO_NBR_OF_AXES] = { 0 };

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t) sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t) sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t) sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

  meanOut->x = (float) sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float) sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float) sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut) {
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z) {
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool sensorsFindBiasValue(BiasObj* bias) {
  static int32_t varianceSampleTime;
  bool foundBias = false;

  // static int cnt = 0;

  if (bias->isBufferFilled) {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

    // if (1) {
    //   DEBUG_PRINT("bias x: %f, y: %f, z: %f\n", (double) bias->variance.x, (double) bias->variance.y, (double) bias->variance.z);
    //   // cnt = 0;
    // }
    if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = bias->mean.x;
      bias->bias.y = bias->mean.y;
      bias->bias.z = bias->mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

bool sensorsBmi270SpiBmp388ManufacturingTest(void) {
  bool testStatus = true;
  if (!accelSelftest()) {
    testStatus = false;
  }

  int8_t accelResult = 0;
  accelResult = bmi2_perform_accel_self_test(&bmi270Dev);
  if (accelResult == BMI2_OK) {
    DEBUG_PRINT("BMI270 accel self-test [OK]\n");
  } else {
    DEBUG_PRINT("BMI270 accel self-test [FAILED]\n");
    testStatus = false;
  }

  return testStatus;
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out) {
  Axis3f rx;
  Axis3f ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

void sensorsBmi270SpiBmp388SetAccMode(accModes accMode) {
  struct bmi2_sens_config sens_cfg;
  switch (accMode) {
    case ACC_MODE_PROPTEST:
      /* set bandwidth and range of accel (740Hz cut-off according to datasheet) */
      sens_cfg.type = BMI2_ACCEL;
      sens_cfg.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
      sens_cfg.cfg.acc.bwp =BMI2_ACC_NORMAL_AVG4;
      sens_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg.cfg.acc.range = BMI2_ACC_RANGE_16G;

      if (bmi2_set_sensor_config(&sens_cfg, 1, &bmi270Dev) != BMI2_OK) {
        DEBUG_PRINT("ACC config [FAIL]\n");
      }

      for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, 500);
      }
      break;
    case ACC_MODE_FLIGHT:
    default:
      /* set bandwidth and range of accel (145Hz cut-off according to datasheet) */
      sens_cfg.type = BMI2_ACCEL;
      sens_cfg.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
      sens_cfg.cfg.acc.bwp =BMI2_ACC_OSR4_AVG1;
      sens_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
      sens_cfg.cfg.acc.range = BMI2_ACC_RANGE_16G;

      if (bmi2_set_sensor_config(&sens_cfg, 1, &bmi270Dev) != BMI2_OK) {
        DEBUG_PRINT("ACC config [FAIL]\n");
      }
      for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
      }
      break;
  }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in) {
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}

void sensorsBmi270SpiBmp388DataAvailableCallback(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  imuIntTimestamp = usecTimestamp();

  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);
  
  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

void __attribute__((used)) BMI270_SPI_TX_DMA_IRQHandler(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(BMI270_SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(BMI270_SPI_TX_DMA_STREAM, BMI270_SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(BMI270_SPI_TX_DMA_STREAM,BMI270_SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(BMI270_SPI, SPI_I2S_DMAReq_Tx, DISABLE);

  // Disable streams
  DMA_Cmd(BMI270_SPI_TX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(spiTxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

void __attribute__((used)) BMI270_SPI_RX_DMA_IRQHandler(void) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  // Stop and cleanup DMA stream
  DMA_ITConfig(BMI270_SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(BMI270_SPI_RX_DMA_STREAM, BMI270_SPI_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(BMI270_SPI_RX_DMA_STREAM, BMI270_SPI_RX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(BMI270_SPI, SPI_I2S_DMAReq_Rx, DISABLE);

  // Disable streams
  DMA_Cmd(BMI270_SPI_RX_DMA_STREAM, DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete

  xSemaphoreGiveFromISR(spiRxDMAComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP388, &isBarometerPresent)
PARAM_GROUP_STOP(imu_sensors)
