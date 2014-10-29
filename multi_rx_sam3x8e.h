//#include "multi_rx2560.h"
//project_Quad 32 bit Arduino Due
//1. stabilized quadrotor
//by: tinnakon kheowree
//0860540582
//tinnakon_za@hotmail.com
//tinnakonza@gmail.com
//clock timer7   (1000 - 2000 us) = (2625 - 5250) = 2625 

//////////////////RC//////////////////////////////////
//#define tarremote 0.025 // fast
//#define tarremote 0.062  //0.092 slow 0.12 0.02 0.08 remote 
//#define tar 0.011 //0.012 0.015
float tarremote = 0.095;//0.065
//////////////////////////////////////////////////

#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900

float CH_THR = 1000.0;
float CH_AIL = 1500.0;
float CH_ELE = 1500.0;
float CH_RUD = 1500.0;
float AUX_1 = 1000.0;
float AUX_2 = 1000.0;
float AUX_3 = 1000.0;
float AUX_4 = 1000.0;

float CH_THRf = 1000.0;
float CH_AILf = 1500.0;
float CH_ELEf = 1500.0;
float CH_RUDf = 1500.0;

int CH_AIL_Cal = 1500;
int CH_ELE_Cal = 1500;
int CH_RUD_Cal = 1500;

int roop = 0;

//RX PIN assignment inside the port
//SET YOUR PINS! TO MATCH RECIEVER CHANNELS
#define CHAN1PIN 62 // RECIEVER 1 PPM ,pin A8
//#define CHAN2PIN 63 // RECIEVER 2
//#define CHAN3PIN 64 // RECIEVER 3
//#define CHAN4PIN 65 // RECIEVER 4
//#define CHAN5PIN 61 // RECIEVER 5
//#define CHAN6PIN 67 //not used at the moment
//#define CHAN7PIN 50 //not used at the moment
//#define CHAN8PIN 51 //not used at the moment

#define ROLL       0
#define PITCH      1
#define YAW        3
#define THROTTLE   2
#define AUX1       5
#define AUX2       4
#define CAMPITCH   6
#define CAMROLL    7

//volatile unsigned long pwmLast[8];
volatile unsigned long last = 0;
uint8_t chan1 = 0;
volatile uint32_t rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

// Arduino Due RX PPM
void pwmHandler(int ch, int pin) {
  ///PPM/////////////////
  uint32_t now, diff;
  //now = micros();
  diff = TC_ReadCV(TC2, 1);
  TC_Stop(TC2, 1);
  TC_Start(TC2, 1);
  last = now;
  if (diff > 7875) chan1 = 0;//3000 us
  else {
    if (2363 < diff && diff < 5775 && chan1 < 8) { //900 - 2200 us
      rcValue[chan1] = diff;
    }
    chan1++;
  }
}
void ch1Handler() {
  pwmHandler(0, CHAN1PIN);
}
//void ch2Handler() { pwmHandler(1, CHAN2PIN); }
//void ch3Handler() { pwmHandler(2, CHAN3PIN); }
//void ch4Handler() { pwmHandler(3, CHAN4PIN); }
//void ch5Handler() { pwmHandler(4, CHAN5PIN); }

void configureReceiver() {
  for (uint8_t chan = 0; chan < 8; chan++) {
    for (uint8_t a = 0; a < 4; a++) {
      rcValue[a] = 1500;
    }
  }
  attachInterrupt(CHAN1PIN, ch1Handler, RISING); //RISING  FALLING CHANGE
  //attachInterrupt(CHAN2PIN,ch2Handler,CHANGE);
  //attachInterrupt(CHAN3PIN,ch3Handler,CHANGE);
  //attachInterrupt(CHAN4PIN,ch4Handler,CHANGE);
  //attachInterrupt(CHAN5PIN,ch5Handler,CHANGE);

  /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);		 // disable write protection for pmc registers
  pmc_enable_periph_clk(ID_TC7);	 // enable peripheral clock TC7
  //TC_CMR_TCCLKS_TIMER_CLOCK1 84Mhz/2 = 42.000 MHz
  //TC_CMR_TCCLKS_TIMER_CLOCK2 84Mhz/8 = 10.500 MHz
  //TC_CMR_TCCLKS_TIMER_CLOCK3 84Mhz/32 = 2.625 MHz
  //TC_CMR_TCCLKS_TIMER_CLOCK4  84Mhz/128 = 656.250 KHz
  /* we want wavesel 01 with RC (timer 7 = TC2,1) (timer 8 = TC2,2)*/
  TC_Configure(/* clock */TC2,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK3);//TC_CMR_TCCLKS_TIMER_CLOCK4
  //TC_SetRC(TC2, 1, 131200);
  TC_Start(TC2, 1);
}

void computeRC() {
  CH_THR = rcValue[THROTTLE]*0.3809523;// 84MHz/32 * 1500 us = 3937.5
  CH_AIL = rcValue[ROLL]*0.3809523;
  CH_ELE = rcValue[PITCH]*0.3809523;
  CH_RUD = rcValue[YAW]*0.3809523;
  AUX_1 = rcValue[AUX1]*0.3809523;
  AUX_2 = rcValue[AUX2]*0.3809523;
  AUX_3 = rcValue[CAMPITCH]*0.3809523;
  AUX_4 = rcValue[CAMROLL]*0.3809523;
  CH_THRf = CH_THRf + (CH_THR - CH_THRf) * 0.02 / tarremote; //Low pass filter
  CH_AILf = CH_AILf + (CH_AIL - CH_AILf) * 0.02 / tarremote; //Low pass filter
  CH_ELEf = CH_ELEf + (CH_ELE - CH_ELEf) * 0.02 / tarremote;
  CH_RUDf = CH_RUDf + (CH_RUD - CH_RUDf) * 0.02 / tarremote;
}
//By tinnakon
void RC_Calibrate() {
  Serial.print("RC_Calibrate"); Serial.println("\t");
  for (int i = 0; i < 10; i++) {
    computeRC();
    delay(20);
  }
  CH_AIL_Cal = CH_AIL;
  CH_ELE_Cal = CH_ELE;
  CH_RUD_Cal = CH_RUD;
  Serial.print(CH_AIL_Cal); Serial.print("\t"); //-0.13
  Serial.print(CH_ELE_Cal); Serial.print("\t"); //-0.10
  Serial.print(CH_RUD_Cal); Serial.println("\t"); //0.03
}
