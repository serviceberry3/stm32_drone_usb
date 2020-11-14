# Make configuration for the Crazyflie 2 platform

PLATFORM_HELP_cf2 = Crazyflie2 platform, includes Crazyflie 2.0, Crazyflie 2.1 and Bolt
PLATFORM_NAME_cf2 = CF2 platform

CPU=stm32f4

CFLAGS += -DDEVICE_TYPE_STRING_FORCE="F407"
# debug print to uart2
CFLAGS += -DENABLE_UART2
CFLAGS += -DDEBUG_PRINT_ON_UART2

######### Sensors configuration ##########
CFLAGS += -DSENSOR_INCLUDED_BMI088_BMP388
PROJ_OBJ += sensors_bmi088_bmp388.o

CFLAGS += -DSENSOR_INCLUDED_MPU9250_LPS25H
PROJ_OBJ += sensors_mpu9250_lps25h.o

CFLAGS += -DSENSOR_INCLUDED_BMI088_SPI_BMP388
PROJ_OBJ += sensors_bmi088_spi_bmp388.o
# should be bmi270_bmp388
CFLAGS += -DSENSOR_INCLUDED_BMI270_SPI_BMP388
PROJ_OBJ += sensors_bmi270_spi_bmp388.o

######### Stabilizer configuration ##########
ESTIMATOR          ?= any
CONTROLLER         ?= Any # one of Any, PID, Mellinger, INDI
POWER_DISTRIBUTION ?= stock
