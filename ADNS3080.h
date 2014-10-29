/*  
//project_Quad 32 bit Arduino Due
//1. stabilized quadrotor 
//by: tinnakon kheowree 
//0860540582
//tinnakon_za@hotmail.com
//tinnakonza@gmail.com
//https://www.facebook.com/tinnakonza

tinnakon Modified from
 *       ADC.cpp - Analog Digital Converter Base Class for Ardupilot Mega
 *       Code by James Goppert. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 */
//#define PIN_SPI_SS0          (77u)
//#define PIN_SPI_SS1          (87u)
//#define PIN_SPI_SS2          (86u)
//#define PIN_SPI_SS3          (78u)
//#define PIN_SPI_MOSI         (75u)
//#define PIN_SPI_MISO         (74u)
//#define PIN_SPI_SCK          (76u)
//#define BOARD_SPI_SS0        (10u)
//#define BOARD_SPI_SS1        (4u)
//#define BOARD_SPI_SS2        (52u)
#define ADNS3080_CHIP_SELECT   53  //SS    10
#define ADNS3080_RESET         49   //RESET  9

byte orig_spi_settings_spcr;
byte orig_spi_settings_spsr;
int _cs_pin=ADNS3080_CHIP_SELECT;

boolean _overflow=false;
boolean _motion=false;
int8_t raw_dx;
int8_t raw_dy;
float GyroXfMO = 0.0;
float GyroYfMO = 0.0;
float change_x;
float change_y;
float posistion_X = 0.0;
float posistion_Y = 0.0;
float vlon, vlat; // position as offsets from original position
unsigned int surface_quality;
float _last_altitude;
int  num_pixels = 30; // number of pixels of resolution in the sensor
//AP_OPTICALFLOW_ADNS3080_SCALER scaler - value returned when sensor is moved equivalent of 1 pixel
float scaler = 1.1;//1.1 , 4.4
// field of view of ADNS3080 sensor lenses AP_OPTICALFLOW_ADNS3080_08_FOV
float field_of_view = 0.202458; // 11.6 degrees
float conv_factor = ((1.0 / (float)(num_pixels * scaler)) * 2.0 * tan(field_of_view / 2.0))*2000.0;        // multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
// 0.00615
float radians_to_pixels = ((num_pixels * scaler) / field_of_view)/20.0;//20.0
//float radians_to_pixels = 0.0;
// 162.99
    
// ADNS3080 hardware config
#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30
#define ADNS3080_CLOCK_SPEED			  24000000

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

// Configuration Bits
#define ADNS3080_LED_MODE_ALWAYS_ON        0x00
#define ADNS3080_LED_MODE_WHEN_REQUIRED    0x01

#define ADNS3080_RESOLUTION_400			400
#define ADNS3080_RESOLUTION_1600		1600

// Extended Configuration bits
#define ADNS3080_SERIALNPU_OFF	0x02

#define ADNS3080_FRAME_RATE_MAX         6469
#define ADNS3080_FRAME_RATE_MIN         2000

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADNS3080 SPECIFIC FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// reset sensor by holding a pin high (or is it low?) for 10us.
void reset()
{
  delay(100);
  digitalWrite(ADNS3080_RESET,HIGH);                 // reset sensor
  delayMicroseconds(100);
  digitalWrite(ADNS3080_RESET,LOW);                  // return sensor to normal
}
// backup_spi_settings - checks current SPI settings (clock speed, etc), sets values to what we need
byte backup_spi_settings()
{
  // store current spi values
  //orig_spi_settings_spcr = SPCR & (DORD | CPOL | CPHA);
  //orig_spi_settings_spsr = SPSR & SPI2X;
  // set the values that we need
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8);// sensor running at 2Mhz.  this is it's maximum speed
  return orig_spi_settings_spcr;
}

// restore_spi_settings - restores SPI settings (clock speed, etc) to what their values were before the sensor used the bus
byte restore_spi_settings()
{
  byte temp;
  // restore SPSR
  //temp = SPSR;
  //temp &= ~SPI2X;
  //temp |= orig_spi_settings_spsr;
  //SPSR = temp;
  // restore SPCR
  //temp = SPCR;
  //temp &= ~(DORD | CPOL | CPHA);   // zero out the important bits
  //temp |= orig_spi_settings_spcr;  // restore important bits
  //SPCR = temp;
  return temp;
}
// write a value to one of the sensor's registers
void write_register(byte address, byte value)
{
  byte junk = 0;
  backup_spi_settings();
  // take the chip select low to select the device
  digitalWrite(_cs_pin, LOW);
  // send register address
  junk = SPI.transfer(address | 0x80 );
  // small delay
  delayMicroseconds(50);
  // send data
  junk = SPI.transfer(value);
  // take the chip select high to de-select:
  digitalWrite(_cs_pin, HIGH);
  restore_spi_settings();
}
// Read a register from the sensor
byte read_register(byte address)
{
  byte result = 0, junk = 0;
  backup_spi_settings();
  // take the chip select low to select the device
  digitalWrite(_cs_pin, LOW);
  // send the device the register you want to read:
  junk = SPI.transfer(address);
  // small delay
  delayMicroseconds(50);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // take the chip select high to de-select:
  digitalWrite(_cs_pin, HIGH);
  restore_spi_settings();
  return result;
}
// init - initialise sensor
// initCommAPI parameter controls whether SPI interface is initialised (set to false if other devices are on the SPI bus and have already initialised the interface)
boolean initOF()
{
  int retry = 0;
  //pinMode(AP_SPI_DATAOUT,OUTPUT);
  //pinMode(AP_SPI_DATAIN,INPUT);
  //pinMode(AP_SPI_CLOCK,OUTPUT);
  pinMode(_cs_pin,OUTPUT);
  pinMode(ADNS3080_RESET,OUTPUT);
  digitalWrite(_cs_pin,HIGH);                 // disable device (Chip select is active low)
  // start the SPI library:
  //SPI.begin();
  // DUE clock divider //
  SPI.setClockDivider(SPI_CLOCK_DIV16);//84 = 1MHZ SPI rate
  // check the sensor is functioning
  if( retry < 3 ) {
    if( read_register(ADNS3080_PRODUCT_ID) == 0x17 )
      return true;
    retry++;
  }
  return false;
}

bool updateOF()
{
  byte motion_reg;
  surface_quality = (unsigned int)read_register(ADNS3080_SQUAL);
  delayMicroseconds(50);  // small delay
  // check for movement, update x,y values
  motion_reg = read_register(ADNS3080_MOTION);
  _overflow = ((motion_reg & 0x10) != 0);  // check if we've had an overflow
  if( (motion_reg & 0x80) != 0 ) {
    raw_dx = ((char)read_register(ADNS3080_DELTA_X));
    delayMicroseconds(50);  // small delay
    raw_dy = ((char)read_register(ADNS3080_DELTA_Y));
    //_motion = true;
  }else{
    raw_dx = 0;
    raw_dy = 0;
    //_motion=false;
  }
  return true;
}
void installation_a3080()
{
  // flowSensor initialization
  if( initOF() == false )
  Serial.println("Failed to initialise ADNS3080");
  delay(1000);
  Serial.println("initialise ADNS3080");
  reset();
  ////// set parameter to  1600 counts per inch////////////////
  byte regVal = read_register(ADNS3080_CONFIGURATION_BITS);
  Serial.println(regVal);
  //regVal &= ~0x10;//400  counts per inch
  //regVal |= 0x10;//1600 counts per inch
  regVal = 80;//80 = 1600 counts per inch ,,, 64 = 400  counts per inch
  delayMicroseconds(100);      // small delay
  write_register(ADNS3080_CONFIGURATION_BITS, regVal);
  delayMicroseconds(100);      // small delay
  regVal = read_register(ADNS3080_CONFIGURATION_BITS);
  Serial.println(regVal);
  delayMicroseconds(100);      // small delay
  ///////////////////////////////////////////////////////////
  regVal = read_register(ADNS3080_EXTENDED_CONFIG);
  delayMicroseconds(50);      // small delay
        // set specific frame period
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);//0xE0 = 2000 ,0xC0 = 5000 Frames/second
        delayMicroseconds(50);          // small delay
        write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x2E);//0x2E = 2000 , 0x12 = 5000 Frames/second
        delayMicroseconds(50);          // small delay
        // decide what value to update in extended config
        //regVal = (regVal & ~0x01);
        regVal = 1;//0 = auto
        // decide what value to update in extended config
        //regVal = (regVal & ~0x01) | 0x01;
    write_register(ADNS3080_EXTENDED_CONFIG, regVal);
    Serial.println(regVal);
    ////////////////////////////////////////////////////////////////
}
void update_positionA3080(float Gyroroll, float Gyropitch, float cos_yaw_x, float sin_yaw_y, float altitude)
{
    // only update position if surface quality is good and angle is not over 45 degrees
    if(surface_quality >= 10) {
        altitude = max(altitude, 0);
        // calculate expected x,y diff due to roll and pitch change
        float exp_change_x = -Gyroroll * radians_to_pixels;
        float exp_change_y = Gyropitch * radians_to_pixels;
        // real estimated raw change from mouse
        change_x = raw_dx - exp_change_x;
        change_y = raw_dy - exp_change_y;
        float avg_altitude = (altitude + _last_altitude)*0.5;
        // convert raw change to horizontal movement in cm
        posistion_X = change_x * avg_altitude * conv_factor;// perhaps this altitude should actually be the distance to the ground?  i.e. if we are very rolled over it should be longer?
        posistion_Y = change_y * avg_altitude * conv_factor;// for example if you are leaned over at 45 deg the ground will appear farther away and motion from opt flow sensor will be less
        // convert x/y movements into lon/lat movement
        vlon = posistion_X * sin_yaw_y + posistion_Y * cos_yaw_x;
        vlat = posistion_Y * sin_yaw_y - posistion_X * cos_yaw_x;
    }
    else{
      posistion_X = 0.0;
      posistion_Y = 0.0;
    }
    _last_altitude = altitude;
    
}
