/*
//Test By tinnakon kheowree
//tinnakon_za@hotmail.com
//tinnakonza@gmail.com
//https://www.facebook.com/tinnakonza

//10/10/2556    write Test_AllSensorV1
//19/10/2556    write TK_Quad32bit_FullSensor_V1
//19/10/2556    write TK_Quad32bit_FullSensor_V2  , PIDA ,  Altitude , Optical Flow Sensor ,CMUCam5 Pixy
//27/10/2556    write TK_Quad32bit_FullSensor_V3  ,Laser Head Transmitter , "BaroSPI_MS5611.h"
//28/10/2556    write TK_Quad32bit_FullSensor_V4  ,read CJMCU117 (not work)
//28/10/2556    write TK_Quad32bit_FullSensor_V5  ,SHARP GP2Y0A02YK0F ,Altitude Hold, Position Hold (indoor),,,Ultrasonic max 1 m change to Baro

support:  Arduino Due
• Atmel SAM3X8E ARM Cortex-M3 CPU 32-bit a 84 MHz clock, ARM core microcontroller
• CJMCU-117 MPU-9250+MS5611 high precision 9 axis 10DOF //SPI 20 MHz 
• MPU6050 Gyro Accelerometer //I2C 400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2
• MS561101BA Barometer//I2C 400kHz
• HMC5883L Magnetometer //I2C 400kHz
• ADNS3080 Optical Flow Sensor//SPI 1 MHz 
• Ultrasonic_HC-SR04
• Distance Measuring Sensor 20-150cm (SHARP GP2Y0A02YK0F)
• GPS NEO-7N //Rx 1 Tx 1 
• GPS NEO-6 //
• CMUCam5 Pixy //Rx 2 Tx 2
• SMD RGB three colors LEDs module //RGB = pin 5,4,3
• Laser Head Transmitter Sensor Module //pin 40

//Motor Redcon 2212 1400 KV 
//ESC Dragon 30A
//propeller 8
----------rx-----------  
A8 = CPPM
 */
#include <Arduino.h>
#include "SPI_sam.h"
#include "MS561101BA32bit.h"
#include "BaroSPI_MS5611.h"
#include "Aconfigsam3x8e.h"
#include "ADNS3080.h"
#include "Wire_due32.h"
#include "multi_rx_sam3x8e.h"
#include "mpu6050sam3x8e.h"
#include "CJMCU117sam3x8e.h"
#include "ahrs_tinsam3x8e.h"
#include "Ultrasonic04.h"
#include "GPSNEO7N_multi.h"
#include "AnalogSource_Arduino.h"
#include "Control_SlidingPIDsam3x8e.h"
#include "motorX4sam3x8e.h"
#include "PixyUART_Due32.h"

///////////////////////////////////////////////////////////////////////////////////////////////
float getAltitude(float pressure2, float temperature2)
{
  //return (1.0f - pow(pressure2/sea_press, 0.190295f)) * 44330.0f;
  //return log(sea_press/pressure2) * (temperaturet+273.15) * 29.271267f; // in meter 
  return ((pow((sea_press/pressure2),1/5.257)-1.0)*(temperature2+273.15))/0.0065;
}
void pushAvg(float val)
{
  movavg_buff[movavg_i] = val;
  movavg_i = (movavg_i + 1) % MOVAVG_SIZE;
}
float getAvg(float * buff, int size)
{
  float sum=0.0;
  for(int i=0;i<size;i++)
  {
    sum += buff[i];
  }
  return sum/size;
}
////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial.print("TK_Quadrotor32Bit_Run_Roop_400Hz");Serial.println("\n");
  pinMode(13, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//pinMode (30, OUTPUT);pinMode (31, OUTPUT);//(13=A=M),(31=B=STABLEPIN),(30=C,GPS FIG LEDPIN)
  digitalWrite(13, HIGH);
  pinMode(Pin_Laser, OUTPUT);//Laser
  pinMode(Pin_LED_R, OUTPUT);
  pinMode(Pin_LED_G, OUTPUT);
  pinMode(Pin_LED_B, OUTPUT);
  digitalWrite(Pin_LED_R, HIGH);
  digitalWrite(Pin_LED_G, LOW);
  digitalWrite(Pin_LED_B, LOW);
  digitalWrite(Pin_Laser, LOW);
  configureReceiver();
  motor_initialize();
  ESC_calibration();
  delay(10);
  CJMCU117_initialize();
  delay(10);
  Wire.begin();
  delay(10);
  mpu6050_initialize();
  delay(10);
  MagHMC5883Int();
  delay(10);
  baro.init(MS561101BA_ADDR_CSB_LOW);
  delay(10);
  installation_a3080();
  delay(10);
  pixyinit();//Serial2.begin(19200);
  delay(10);
  UltrasonicInt();
  delay(10);
  GPS_multiInt();
  delay(10);
     for(uint8_t i=0; i<100; i++) 
    {
     mpu6050_Gyro_Values();
     mpu6050_Accel_Values();
     Mag5883Read();
     temperaturetr = baro.getTemperature(MS561101BA_OSR_4096);
     presser = baro.getPressure(MS561101BA_OSR_4096);
     pushAvg(presser);
     delay(10);
    }
    sea_press = presser + 0.09;//presser 1007.25   1003.52
    Serial.print("Temperature ");Serial.print(temperaturetr);
    Serial.print(" presser ");Serial.println(sea_press);
    sensor_Calibrate();//sensor.h
    mpu6050_Get_accel();
    mpu6050_Get_gyro();
    ahrs_initialize();//ahrs.h
    RC_Calibrate();//"multi_rxPPM2560.h"
  delay(10);
  digitalWrite(13, LOW);
  digitalWrite(Pin_LED_R, LOW);
  digitalWrite(Pin_LED_G, LOW);
  digitalWrite(Pin_LED_B, LOW);
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop() {
  while(1){
        while(Serial1.available())
   {
     //digitalWrite(30, LOW);//C
     char byteGPS1=Serial1.read(); 
     GPS_UBLOX_newFrame(byteGPS1);
     GPS_LAT1 = GPS_coord[LAT]/10000000.0;// 1e-7 degrees / position as degrees (*10E7)
     GPS_LON1 = GPS_coord[LON]/10000000.0;
     if(GPS_FIX  == 1 && GPS_Present == 1){
        digitalWrite(Pin_LED_R, HIGH);
       }
       else{
         digitalWrite(Pin_LED_R, LOW);
       }
     }//end gps
/////////CMUCam5 Pixy///////////////////////////////
      Read_pixy();
///////////////////////////////////////
    Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor/////////
    if(Dt_sensor <= 0){Dt_sensor = 1001;}
    if(Dt_sensor >= 1000 && gyroSamples < 2)////Collect 2 samples = 1000 us 
    {  
        sensorPreviousTime = micros();
        mpu6050_readGyroSum();
        mpu6050_readAccelSum();
    }
   Dt_roop = micros() - previousTime;// 200 Hz task loop (5 ms)  , 2500 us = 400 Hz
   if(Dt_roop <= 0){Dt_roop = 2501;}  
   if (Dt_roop >= 2500) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
/////get sensor////////////////////////////////////////////////////////////
      //mpu6050_Get_accel();
      mpu6050_Get_gyro();
      //mpu9250_Get();
//////////////////////////////////////////////////////////
  if (frameCounter % TASK_100HZ == 0)// 100 Hz tak
 {
  //data_interrupt();
  analogReadSUM();//AnalogSource_Arduino.h
  presser = baro.getPressure(MS561101BA_OSR_4096);
  pushAvg(presser);
 }
////////////////Moving Average Filters///////////////////////////
      GyroXf = (GyroX + GyroX2)/2.0;
      GyroYf = (GyroY + GyroY2)/2.0;
      GyroZf = (GyroZ + GyroZ2)/2.0;
      //AccXf = (AccX + AccX2)/2.0;
      //AccYf = (AccY + AccY2)/2.0;
      //AccZf = (AccZ + AccZ2)/2.0;
      //AccX2 = AccX;AccY2 = AccY;AccZ2 = AccZ;//acc Old1
      GyroX2 = GyroX;GyroY2 = GyroY;GyroZ2 = GyroZ;//gyro Old1
////////////////Low pass filter/////////////////////////////////
      GyroXfMO = GyroXfMO + (GyroXf - GyroXfMO)*19.6*G_Dt; //29 - 49.4
      GyroYfMO = GyroYfMO + (GyroYf - GyroYfMO)*19.6*G_Dt;//39.6
      //GyroZf = GyroZf + (GyroZ - GyroZf)*49.6*G_Dt;
    AccXf = AccXf + (AccX - AccXf)*0.121;//12.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
    AccYf = AccYf + (AccY - AccYf)*0.121;//12.4
    AccZf = AccZf + (AccZ - AccZf)*0.121;//12.4
//////////////////////////////////////////////////////////
    ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, c_magnetom_x, c_magnetom_y, c_magnetom_z, G_Dt);//quaternion ,direction cosine matrix ,Euler angles
//Observer kalman filter////////////////////////////////////////////////////////////////////////////////////////////////
//A=[1 0.01;0 1]; //B=[0;0.01];  //C = [1 0];
//Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y');
//Q = 0.1; % A number greater than zero
//R = 0.135; % A number greater than zero
//[kalmf,L,P,M,Z] = kalman(Plant,Q,R); //%kalmf = kalmf(1,:); //M,   % innovation gain

//Predicted (a priori) state estimate
//u_z = (motor_FrontLf + motor_FrontRf + motor_BackLf + motor_BackRf);//uz - g ,,unit N *0.001691) - 9.81
u_z = accrZ_Earth;
z2_hat2 = z2_hat + u_z*G_Dt;//z2_hat = velocity
z1_hat2 = z1_hat + z2_hat2*G_Dt;//z1_hat = Altitude
//z1_hat = constrain(z1_hat, 0, 100);//0 - 100 m
///////////////////////////////////////////////////////////////////
//Updated (a posteriori) state estimate
//Update estimate with measurement zk
z1_hat = z1_hat2 + 0.0453*(Altitude_Baro_ult - z1_hat2);//K1 =0.2887  0.09187 0.01187 0.0140
z2_hat = z2_hat2 + 0.0086*(Altitude_Baro_ult - z1_hat2) + 0.0045*(Vz_Baro_ult - z2_hat2);//K2 =0.0487  0.02487 0.01087 0.0099
baro_vz = (z1_hat - baro_vz_old)/G_Dt;
baro_vz_old = z1_hat;
/////////////////////////////////////////////////////////////////////////////////////////////////
//Sliding modeControl/////////////////////////////////////
    Control_SlidRate();//"Control_Slid.h"
//////Out motor///////////
//armed = 1;
    motor_Mix();//"motor.h"
/////////////////////////
    motor_command(); 
////////end Out motor//////
  if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
 {
  mpu6050_Get_accel();
  computeRC();//multi_rx.h
    if (CH_THR < MINCHECK)  //////ARM and DISARM your Quadrotor///////////////
        {
            if (CH_RUD > MAXCHECK && armed == 0 && abs(ahrs_p) < 5 && abs(ahrs_r) < 5)//+- 5 deg, ARM 
            {
                armed = 1;
                //digitalWrite(31, HIGH);//B
                digitalWrite(Pin_LED_B, HIGH);
                Altitude_Ground = Altitude_baro;
            }
            if (CH_RUD < MINCHECK && armed == 1) //DISARM
            {
                armed = 0;
                //digitalWrite(31, LOW);
                digitalWrite(Pin_LED_B, LOW);
            }
            if (CH_RUD < MINCHECK && armed == 0 && CH_ELE > MAXCHECK) //Mag_Calibrate
            {
              Mag_Calibrate();//#include "mpu6050.h"
            }
        }//end  ARM and DISARM your helicopter/////////////// 
 }
      
      if (frameCounter % TASK_20HZ == 0)// 20 Hz task (50 ms)
        {
          Chack_Command();
          UltrasonicRead();//"Ultrasonic04.h"
          Mag5883Read();
          //updateOF();//AP_OPTICALFLOW_ADNS3080
          //Get_pixy();	
          //update_positionA3080(GyroXfMO ,GyroYfMO ,1.0 ,0.0 ,z1_hat);//(float Gyroroll, float Gyropitch, float cos_yaw_x, float sin_yaw_y, float altitude)
          Distance_Measuring_Get();//AnalogSource_Arduino.h
          Control_PositionHold();//#include "Control_SlidingPIDsam3x8e.h"
        }
        if (frameCounter % TASK_10HZ == 0)//roop TASK_10HZ
        {
          presserf = getAvg(movavg_buff, MOVAVG_SIZE);
          Altitude_baro = getAltitude(presserf,temperaturetr);//Altitude_Ground
          Altitude_barof = Altitude_baro - Altitude_Ground + Altitude_II;
        }
      if (frameCounter % TASK_10HZ == 0)//roop print  ,TASK_5HZ  TASK_10HZ
        {
            //Serial.print(CH_THR);Serial.print("\t");
            //Serial.print(CH_AIL);Serial.print("\t");  
            //Serial.print(CH_ELE);Serial.print("\t");
            //Serial.print(CH_RUD);Serial.print("\t");  
            Serial.print(AUX_1,0);Serial.print("\t"); 
            //Serial.print(AUX_2);Serial.print("\t"); 
            //Serial.print(AUX_3);Serial.print("\t"); 
            //Serial.print(AUX_4);Serial.print("\t"); 
            //Serial.print(failsafeCnt);Serial.print("\t");
            
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            //Serial.print(setpoint_rate_pitch);Serial.print("\t"); 
             
            //Serial.print(MagX1);Serial.print("\t");
            //Serial.print(MagXf);Serial.print("\t");
            //Serial.print(MagY1);Serial.print("\t");
            //Serial.print(MagYf);Serial.print("\t");
            //Serial.print(MagZ1);Serial.print("\t");  
            //Serial.print(MagZf);Serial.print("\t");
            
            //Serial.print(c_magnetom_x);Serial.print("\t");
            //Serial.print(c_magnetom_y);Serial.print("\t");
            //Serial.print(c_magnetom_z);Serial.print("\t"); 
            
            //Serial.print(GPS_FIX1);Serial.print("\t");
            //Serial.print(GPS_LAT1,9);Serial.print("\t"); 
            //Serial.print(GPS_LAT1f,9);Serial.print("\t");

            //Serial.print(GPS_LON1,9);Serial.print("\t");
            //Serial.print(GPS_LON1f,9);Serial.print("\t");
            //Serial.print(GPS_LON1f2,9);Serial.print("\t");
            //Serial.print(error_LAT);Serial.print("\t");
            //Serial.print(error_LON);Serial.print("\t");
            //Serial.print(GPS_speed);Serial.print("\t");//cm/s
            //Serial.print(GPS_ground_course);Serial.print("\t");//deg
            
            //Serial.print(_velocity_north);Serial.print("\t");
            //Serial.print(actual_speedX);Serial.print("\t");
            //Serial.print(actual_speedXf);Serial.print("\t");
            //Serial.print(vx_hat);Serial.print("\t");
            //Serial.print(_velocity_east);Serial.print("\t");
            //Serial.print(_vel_down);Serial.print("\t");
            //Serial.print(actual_speedY);Serial.print("\t");
            //Serial.print(actual_speedYf);Serial.print("\t");
            //Serial.print(vy_hat);Serial.print("\t");
            //Serial3.print(GPS_Distance);Serial3.print("\t");
            //Serial3.print(GPS_ground_course);Serial3.print("\t");
            //Serial3.print(Control_XEf);Serial3.print("\t");
            //Serial3.print(Control_YEf);Serial3.print("\t");
            //Serial3.print(Control_XBf);Serial3.print("\t");
            //Serial3.print(Control_YBf);Serial3.print("\t");
            
            //Serial.print(Control_XEf);Serial.print("\t");
            //Serial.print(Control_YEf);Serial.print("\t");
            Serial.print(Control_XBf);Serial.print("\t");
            Serial.print(Control_YBf);Serial.print("\t");
            
            //Serial.print(panError);Serial.print("\t");
            //Serial.print(tiltError);Serial.print("\t");
            //Serial.print(LError);Serial.print("\t");
            
            //Serial.print(posistion_X);Serial.print("\t");
            //Serial.print(posistion_Y);Serial.print("\t");
            //Serial.print(surface_quality);Serial.print("\t");
            
            //Serial.print(Distance_L);Serial.print("\t");
            //Serial.print(Distance_R);Serial.print("\t");
            //Serial.print(Distance_F);Serial.print("\t");
            //Serial.print(Distance_B);Serial.print("\t");
            //Serial.print(Distance_X);Serial.print("\t");
            //Serial.print(Distance_Y);Serial.print("\t");
            //Serial.print(V_Bat);Serial.print("\t");
            
            //Serial.print(TempMPU);Serial.print("\t");
            //Serial.print(temperaturetr);Serial.print("\t");
            //Serial.print(presser,3);Serial.print("\t");
            //Serial.print(presserf,3);Serial.print("\t");
            //Serial.print(Altitude_baro);Serial.print("\t");
            //Serial.print(Altitude_barof);Serial.print("\t");
            //Serial.print(z1_hat);Serial.print("\t"); 
            //Serial.print(baro_vz);Serial.print("\t");
            //Serial.print(vz_sonaf);Serial.print("\t");
            //Serial.print(z2_hat);Serial.print("\t");
            //Serial.print(Altitude_sona);Serial.print("\t");
            //Serial.print(Altitude_sonaf);Serial.print("\t");
            //Serial.print(Altitude_hat);Serial.print("\t");
            //Serial.print(vz_sona*10);Serial.print("\t");
            //Serial.print(vz_hat*10);Serial.print("\t");
            //Serial.print(h_counter);Serial.print("\t");
            //Serial.print(GPS_hz);Serial.print("\t"); 

            //Serial.print(vz_hat);Serial.print("\t");
            //Serial.print(DCM10);Serial.print("\t");
            //Serial.print(DCM11);Serial.print("\t");
            //Serial.print(DCM12);Serial.print("\t");
            //Serial.print(AccX);Serial.print("\t");
            //Serial.print(AccXf);Serial.print("\t");
            //Serial.print(AccXf2);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");  
            //Serial.print(AccYf);Serial.print("\t"); 
            //Serial.print(AccZ,3);Serial.print("\t");
            //Serial.print(AccZf2,3);Serial.print("\t");
            //Serial.print(AccZf3,3);Serial.print("\t");
            //Serial.print(AccZf);Serial.print("\t");       
            //Serial.print(accrX_Earth);Serial.print("\t");
            //Serial.print(accrY_Earth);Serial.print("\t");
            //Serial.print(accrZ_Earth);Serial.print("\t");
            
            //Serial.print(accelRaw[XAXIS]);Serial.print("\t");
            //Serial.print(accelRaw[YAXIS]);Serial.print("\t");
            //Serial.print(accelRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(-GyroX*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroXf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(roll_D_rate);Serial.print("\t");
            //Serial.print(GyroY*RAD_TO_DEG,3);Serial.print("\t");
            //Serial.print(GyroXfMO*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
            //Serial.print(gyroRaw[XAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[YAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(_accel9250X);Serial.print("\t"); 
            //Serial.print(_accel9250Y);Serial.print("\t");
            //Serial.print(_accel9250Z);Serial.print("\t");
            
            Serial.print(ahrs_r);Serial.print("\t");
            Serial.print(ahrs_p);Serial.print("\t");  
            Serial.print(ahrs_y);Serial.print("\t");  
            //Serial3.print(ahrs_y*RAD_TO_DEG);Serial3.print("\t"); 
            //Serial.print(cos_rollcos_pitch);Serial.print("\t"); 
             
            //Serial.print(x_angle);Serial.print("\t");
            
            //Serial.print(err_pitch_rate);Serial.print("\t");
            //Serial.print(roll_I_rate);Serial.print("\t");
            
            //Serial.print(u2_roll);Serial.print("\t");
            //Serial.print(u3_pitch);Serial.print("\t");
            //Serial.print(u4_yaw);Serial.print("\t");
            
            //Serial.print(motor_FrontL);Serial.print("\t");
            //Serial.print(motor_FrontLf);Serial.print("\t");
            //Serial.print(motor_FrontR);Serial.print("\t");
            //Serial.print(motor_FrontRf);Serial.print("\t");
            //Serial.print(motor_BackL);Serial.print("\t");
            //Serial.print(motor_BackLf);Serial.print("\t");
            //Serial.print(motor_BackR);Serial.print("\t");
            //Serial.print(motor_BackRf);Serial.print("\t");
            //Serial.print(motor_Left);Serial.print("\t");
            //Serial.print(motor_Right);Serial.print("\t");
            
            Serial.print(GPS_numSat);Serial.print("\t");
            //Serial.print(gyroSamples2);Serial.print("\t");
            Serial.print(1/G_Dt);Serial.print("\t");
            Serial.print("Hz");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
        }//end roop 5 Hz 
      if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            //time_auto++;
            temperaturetr = baro.getTemperature(MS561101BA_OSR_4096);
            //mpu6050_Temp_Values();
            Remote_TrimACC();//motor.h
            if(Status_LED == LOW)
             {
              Status_LED = HIGH;
              digitalWrite(Pin_LED_G, LOW);
              }
            else
            {
            Status_LED = LOW;
            digitalWrite(Pin_LED_G, HIGH);
            }
            digitalWrite(13, Status_LED);//A
        }//end roop 1 Hz
    }//end roop 400 Hz
  }//end while
}
