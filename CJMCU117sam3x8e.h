//project_Quad 32 bit Arduino Due
//1. stabilized quadrotor 
//by: tinnakon kheowree 
//0860540582
//tinnakon_za@hotmail.com
//tinnakonza@gmail.com
//CJMCU-117 MPU-9250+MS5611 high precision 9 axis 10DOF //SPI 1 MHz 
//https://github.com/BeaglePilot/ardupilot/tree/master/libraries

#define MPU9250_CS_PIN 48 //pin 51 connected to mpu9250's chip select pin
#define DMP_FIFO_BUFFER_SIZE 72        // DMP FIFO buffer size

#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05
// MPU9250 registers
#define MPUREG_XA_OFFS_H                                0x77    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x78    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x7A    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x0B    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x0D    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x0E    // Z axis accelerometer offset (low byte)
// MPU6000 & MPU9250 registers
// not sure if present in MPU9250
// #define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#define MPUREG_SMPLRT_1000HZ                             0x00
#define MPUREG_SMPLRT_500HZ                              0x01
#define MPUREG_SMPLRT_250HZ                              0x03
#define MPUREG_SMPLRT_200HZ                              0x04
#define MPUREG_SMPLRT_100HZ                              0x09
#define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG                                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#define BITS_GYRO_FS_250DPS                              0x00
#define BITS_GYRO_FS_500DPS                              0x08
#define BITS_GYRO_FS_1000DPS                             0x10
#define BITS_GYRO_FS_2000DPS                             0x18
#define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#define BITS_GYRO_YGYRO_SELFTEST                 0x40
#define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG                             0x1C
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                                  0x23
#define MPUREG_INT_PIN_CFG                              0x37
#define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#define BIT_LATCH_INT_EN                                 0x20    // latch data ready pin
#define MPUREG_INT_ENABLE                               0x38
// bit definitions for MPUREG_INT_ENABLE
#define BIT_RAW_RDY_EN                                   0x01
#define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#define BIT_UNKNOWN_INT_EN                               0x04
#define BIT_I2C_MST_INT_EN                               0x08
#define BIT_FIFO_OFLOW_EN                                0x10
#define BIT_ZMOT_EN                                              0x20
#define BIT_MOT_EN                                               0x40
#define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS                               0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#define BIT_RAW_RDY_INT                                  0x01
#define BIT_DMP_INT                                              0x02
#define BIT_UNKNOWN_INT                                  0x04
#define BIT_I2C_MST_INT                                  0x08
#define BIT_FIFO_OFLOW_INT                               0x10
#define BIT_ZMOT_INT                                             0x20
#define BIT_MOT_INT                                              0x40
#define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A
// bit definitions for MPUREG_USER_CTRL
#define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable hal.spi->interface
#define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                                 0x6D            // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E            // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                                  0x6F            // DMP related register
#define MPUREG_DMP_CFG_1                                0x70            // DMP related register
#define MPUREG_DMP_CFG_2                                0x71            // DMP related register
#define MPUREG_FIFO_COUNTH                              0x72
#define MPUREG_FIFO_COUNTL                              0x73
#define MPUREG_FIFO_R_W                                 0x74
#define MPUREG_WHOAMI                                   0x75
// Configuration bits MPU 3000, MPU 6000 and MPU9250
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07
/*
 *  PS-MPU-9250A-00.pdf, page 8, lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
#define GYRO_SCALE (0.0174532f / 16.4f)
/*
 *  PS-MPU-9250A-00.pdf, page 9, lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPUXk
 *  variants however
 */
// MPU6000 accelerometer scaling
#define MPU9250_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

int16_t  _gyro9250X;
int16_t  _gyro9250Y;
int16_t  _gyro9250Z;
int16_t  _accel9250X;  
int16_t  _accel9250Y;
int16_t  _accel9250Z;
int32_t  _accel_sumX;
int32_t  _accel_sumY;
int32_t  _accel_sumZ;
int32_t  _gyro_sumX;
int32_t  _gyro_sumY;
int32_t  _gyro_sumZ;
uint8_t  _sum_count;
    
static int16_t spi_transfer_16(void)
{
    uint8_t byte_H, byte_L;
    byte_H = SPI.transfer(0);
    byte_L = SPI.transfer(0);
    return (((int16_t)byte_H)<<8) | byte_L;
}
uint8_t register_read( uint8_t reg )
{
    uint8_t return_value;
    uint8_t addr = reg | 0x80; // Set most significant bit
    digitalWrite(MPU9250_CS_PIN, LOW);
    SPI.transfer(addr);
    delayMicroseconds(50);
    return_value = SPI.transfer(0);
    digitalWrite(MPU9250_CS_PIN, HIGH);
    return return_value;
}
void register_write(uint8_t reg, uint8_t val)
{
    digitalWrite(MPU9250_CS_PIN, LOW);
    SPI.transfer(reg);
      // small delay
    //delayMicroseconds(50);
    SPI.transfer(val);
    digitalWrite(MPU9250_CS_PIN, HIGH);
}
void data_interrupt(){
  //* one resister address followed by seven 2-byte registers
  digitalWrite(MPU9250_CS_PIN, LOW);
  SPI.transfer(MPUREG_ACCEL_XOUT_L  | 0x80 | 0x40);// // SPI read, autoincrement
  //SPI.transfer(0);
  //SPI.transfer(0);
  //SPI.transfer(0);
  //delayMicroseconds(50);  // small delay
  int16_t  accel_sX  = spi_transfer_16();
  int16_t  accel_sY  = spi_transfer_16();
  int16_t  accel_sZ  = spi_transfer_16();
  digitalWrite(MPU9250_CS_PIN, HIGH);
  //delayMicroseconds(50);  // small delay
  digitalWrite(MPU9250_CS_PIN, LOW);
  SPI.transfer(MPUREG_GYRO_XOUT_L  | 0x80 | 0x40);// // SPI read, autoincrement
  //delayMicroseconds(50);  // small delay
  int16_t  gyro_sX  = spi_transfer_16();
  int16_t  gyro_sY  = spi_transfer_16();
  int16_t  gyro_sZ  = spi_transfer_16();
  digitalWrite(MPU9250_CS_PIN, HIGH);
  _accel_sumX += accel_sX;
  _accel_sumY += accel_sY;
  _accel_sumZ += accel_sZ;
  _gyro_sumX += gyro_sX;
  _gyro_sumY += gyro_sY;
  _gyro_sumZ += gyro_sZ;
  _sum_count++;
}

void mpu9250_Get(){
  if(_sum_count == 0){
    _sum_count = 1;
  }
_accel9250X = _accel_sumX / _sum_count;  
_accel9250Y = _accel_sumY / _sum_count;
_accel9250Z = _accel_sumZ / _sum_count;
_gyro9250X = _gyro_sumX / _sum_count;
_gyro9250Y = _gyro_sumY / _sum_count;
_gyro9250Z = _gyro_sumZ / _sum_count;
    // Reset SUM variables
  _accel_sumX = 0;
  _accel_sumY = 0;
  _accel_sumZ = 0;
  _gyro_sumX = 0;
  _gyro_sumY = 0;
  _gyro_sumZ = 0;
  _sum_count = 0;
}
void CJMCU117_initialize()
{
    Serial.println("CJMCU117_initialize");
    // MPU9250 chip select setup
    pinMode(MPU9250_CS_PIN, OUTPUT);
    digitalWrite(MPU9250_CS_PIN, HIGH);  // disable device (Chip select is active low)
    delay(1);              
    // start the SPI library:
    SPI.begin();
    // DUE clock divider //
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);//0 3
    SPI.setClockDivider(SPI_CLOCK_DIV16);//84 = 1MHZ SPI rate
    delayMicroseconds(50);
    uint8_t whoami = register_read(MPUREG_WHOAMI);
    Serial.println(whoami,HEX);
    if (whoami != 0x71) {
        // TODO: we should probably accept multiple chip
        // revisions. This is the one on the PXF
        Serial.println("CJMCU117_initialize_bad WHOAMI");
    }
    // Chip reset
    uint8_t tries;
    for (tries = 0; tries<5; tries++) {
        register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);//register_write
        delay(100);
        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        delay(5);
        // check it has woken up
        if (register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }
    }
      if (tries == 5) {
        Serial.println("Failed to boot MPU9250 5 times");
    }
    register_write(MPUREG_PWR_MGMT_2, 0x00);// only used for wake-up in accelerometer only low power mode
    // Disable I2C bus (recommended on datasheet)
    register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);
    // used a fixed filter of 42Hz on the sensor, then filter using
    // the 2-pole software filter
    register_write(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
    // set sample rate to 1kHz, and use the 2 pole filter to give the
    // desired rate
    register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_1000HZ);
    register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s
    // RM-MPU-9250A-00.pdf, pg. 15, select accel full scale 8g
    register_write(MPUREG_ACCEL_CONFIG,2<<3);
    // configure interrupt to fire when new data arrives
    register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);
    // now that we have initialised, we set the SPI bus speed to high
    // (8MHz on APM2)
    //SPI.setClockDivider(SPI_CLOCK_DIV8);//2MHZ SPI rate
    //digitalWrite(MPU9250_CS_PIN, LOW);
    //attachInterrupt(6,data_interrupt,RISING);//
}
void mpu9250_Gyro_Values()
{
   
}	
void mpu9250_Accel_Values()
{
    
}
void mpu9250_readGyroSum() {
    mpu9250_Gyro_Values();
    //gyroSum[XAXIS] += gyroRaw[XAXIS];
    //gyroSum[YAXIS] += gyroRaw[YAXIS];
    //gyroSum[ZAXIS] += gyroRaw[ZAXIS];
    //gyroSamples++;
}
void mpu9250_Get_gyro()
{       
    // Calculate average
    //GyroX = (gyroSum[XAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetX;
    //GyroY = (gyroSum[YAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetY;
    //GyroZ = (gyroSum[ZAXIS] / gyroSamples)*gyroScaleFactor - gyro_offsetZ;            
    // Reset SUM variables
    //gyroSum[XAXIS] = 0;
    //gyroSum[YAXIS] = 0;
    //gyroSum[ZAXIS] = 0;
    //gyroSamples2 = gyroSamples;
    //gyroSamples = 0;            
}

void mpu9250_readAccelSum() {
    mpu9250_Accel_Values();
    //accelSum[XAXIS] += AccXm;
    //accelSum[YAXIS] += AccYm;
    //accelSum[ZAXIS] += AccZm;  
    //accSamples++;
}
void mpu9250_Get_accel()
{
    // Calculate average
    //AccX = (accelSum[XAXIS] / accSamples)*accelScaleFactor - acc_offsetX;
    //AccY = (accelSum[YAXIS] / accSamples)*accelScaleFactor - acc_offsetY;
    //AccZ = (accelSum[ZAXIS] / accSamples)*accelScaleFactor;          
    // Apply correct scaling (at this point accel reprensents +- 1g = 9.81 m/s^2)
    // Reset SUM variables
    //accelSum[XAXIS] = 0;
    //accelSum[YAXIS] = 0;
    //accelSum[ZAXIS] = 0; 
    //accSamples = 0;   
}
void sensor9250_Calibrate()
{
  Serial.print("Sensor9250_Calibrate");Serial.println("\t");
    for (uint8_t i=0; i<60; i++) //Collect 60 samples
    {
        Serial.print("- ");
        mpu9250_readGyroSum();
        mpu9250_readAccelSum();
        digitalWrite(13, HIGH);
        delay(15);
        digitalWrite(13, LOW);
        delay(15);
    }
    Serial.println("- ");
    //gyro_offsetX = (gyroSum[XAXIS]/gyroSamples)*gyroScaleFactor;
    //gyro_offsetY = (gyroSum[YAXIS]/gyroSamples)*gyroScaleFactor;
    //gyro_offsetZ = (gyroSum[ZAXIS]/gyroSamples)*gyroScaleFactor;
    //acc_offsetX = (accelSum[XAXIS]/gyroSamples)*accelScaleFactor;
    //acc_offsetY = (accelSum[YAXIS]/gyroSamples)*accelScaleFactor;
    //acc_offsetZ = (accelSum[ZAXIS]/gyroSamples)*accelScaleFactor;
    //AccZf = acc_offsetZ;//15.4
    //gyroSamples = 0.0;
    Serial.print("GYRO_Calibrate");Serial.print("\t");
    Serial.print(gyro_offsetX);Serial.print("\t");//-0.13
    Serial.print(gyro_offsetY);Serial.print("\t");//-0.10
    Serial.print(gyro_offsetZ);Serial.println("\t");//0.03 
    Serial.print("ACC_Calibrate");Serial.print("\t");
    Serial.print(acc_offsetX);Serial.print("\t");
    Serial.print(acc_offsetY);Serial.print("\t");
    Serial.print(acc_offsetZ);Serial.println("\t"); 
    //gyro_offsetX = -0.03;//-0.03
    //gyro_offsetY = -0.10;//-0.10
    //gyro_offsetZ = 0.01;//0.00
    //acc_offsetX = -0.08;//-0.08 -0.18 0.11 -0.36  Trim PITCH CONTROL   -10.07	-10.55	-9.82
    //acc_offsetY = -0.25;//-0.35 0.16 -0.14 0.18 Trim ROLL CONTROL     10.39	9.74	11
    //acc_offsetZ = 0.0;//0.245 0.235 10.2
}
