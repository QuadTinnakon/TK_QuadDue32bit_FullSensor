//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
#define X_CENTER    160L
#define Y_CENTER    100L
#define Z_CENTER    6000
int32_t panError, tiltError, LError;

#define PIXY_INITIAL_ARRAYSIZE      30 //30
#define PIXY_MAXIMUM_ARRAYSIZE      130 //130
#define PIXY_START_WORD             0xaa55
#define PIXY_START_WORDX            0x55aa
#define PIXY_DEFAULT_ADDR           0x54  // I2C

uint16_t signature;
uint16_t xblock;
uint16_t yblock;
uint16_t width;
uint16_t height;
uint16_t words_Pixy[15];
  
  void pixyinit()
  {
    //Serial2.begin(19200);
  }
  
   void Read_pixy()
  {
    
  }
  
     void Get_pixy()
  {
      panError = X_CENTER - xblock;
      tiltError = yblock - Y_CENTER;
      LError = Y_CENTER - (width * height)*0.5;
  }
