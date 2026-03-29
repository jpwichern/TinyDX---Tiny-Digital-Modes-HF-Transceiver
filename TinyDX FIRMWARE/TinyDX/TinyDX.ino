//*********************************************************************************************************
//**********************  TinyDX  - QRPp DIGITAL MODES MULTIBAND HF TRANSCEIVER   *****************
//********************************* Write up start: 06/01/2024 ********************************************
// FIRMWARE VERSION: TinyDX_V1.5PG8W
// Barb(Barbaros ASUROGLU) - WB2CBA - 2025
// Jiri Wichern - PG8W - 2026
//*********************************************************************************************************
// Required Libraries
// ----------------------------------------------------------------------------------------------------------------------
// Etherkit Si5351 (Needs to be installed via Library Manager to arduino ide) - SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//*****************************************************************************************************
// IMPORTANT NOTE: Use V2.1.3 or above of NT7S SI5351 Library!
// Fixes from V1.4:
// - set_freq_manual needs a PLL frequency that's between 600 and 900 MHz and 4, 6 or 8 times the output frequency. This plays a role when selecting the 10 meters band.
// - Also, after set_freq_manual, the PLL that's set needs a reset to make sure it tunes to the requested frequency.
// - Removed some unused global variables.
// - SI5351 now only updates frequencies when there is an actual change. So no longer every loop.
// Features from V1.4:
// - Switching in normal mode now gives a response of one short blink of the TX led on frequency change.
// - Extended switch interface for selecting all 5 bands and 6 different modes. Simultaniously switch both switches to enter or exit the mode.
//   When entering the mode, TX led blinks 2 short 2 long and then the status of the band and mode counters: short blinks for the counts with one long blink in between.
//   Counts can be 0 if no selection has been made before. If no selection has been made at all, you thus see three long blinks in a row.
//   You then have a 5 second period to toggle a switch. Either band, or mode. Each toggle shifts to the next band or mode.
//   After each toggle you have another 5 seconds and the toggle count will be displayed with one long blink followed by short blinks for the count.
//   After that time, any switching returns the TinyDX switch interface to 'normal' mode, and the switches will adhere to the compiled-in bands and modes selections.
//*****************************************************************************************************

//*************************************[ LICENCE and CREDITS ]*********************************************
//  Initial FSK TX Signal Generation code by: Burkhard Kainka(DK7JD) - http://elektronik-labor.de/HF/SDRtxFSK2.html
//  SI5351 Library by Jason Mildrum (NT7S) - https://github.com/etherkit/Si5351Arduino
//  Improved FSK TX  signal generation code is from JE1RAV https://github.com/je1rav/QP-7C
//
// License
// -------
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


//*******************************[ LIBRARIES ]*************************************************
#include <si5351.h>
#include "Wire.h"
#include <EEPROM.h>
//*******************************[ VARIABLE DECLERATIONS ]*************************************
unsigned long int freq = 14074000;
int32_t cal_factor = 0;
int Osc_State = 1; //At start oscillator is in undefined state, so force update

unsigned long F_FT8;
unsigned long F_FT4;
unsigned long F_FT2;
unsigned long F_JT9;
unsigned long F_JT65;
unsigned long F_WSPR;

unsigned long B1_FT8;
unsigned long B1_FT4;
unsigned long B2_FT8;
unsigned long B2_FT4;

uint32_t ReferenceFrequency = 26000000; //TCXO Frequency used as a reference for the Si5351PLL

unsigned long prevTime = 0;
unsigned int prevBtns = 0, bandPushCount = 0, modePushCount = 0;

// **********************************[ DEFINE's ]***********************************************
#define BAND1 20 // Change to your preferred band 20m/17m/15m/12m/10m
#define BAND2 10 // Change to your preferred band 20m/17m/15m/12m/10m
#define TX 13 //TX LED
#define TXSW 9 //TX BOOST SUPPLY
#define RX  8 // RX SWITCH
#define M_SW  5 // MODE SWITCH
#define B_SW  4 // BAND SWITCH
Si5351 si5351;

//*************************************[ SETUP FUNCTION ]************************************** 
void setup()
{
  Wire.begin(); // wake up I2C bus 
  
  Serial.begin(9600); while (!Serial);
       
  pinMode(TX, OUTPUT);

  pinMode(TXSW, OUTPUT);

  pinMode(RX, OUTPUT);
        
  pinMode(M_SW, INPUT);

  pinMode(B_SW, INPUT);

  digitalWrite(TX,0);
      
  digitalWrite(TXSW,0);

  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0

  //FLASH RED TX LED FOR POWER ON INDICATION
  Blink_TX_Led_Long(3);   

  SW_assign();  
                              
//------------------------------- SET SI5351 VFO -----------------------------------  
  // The crystal load value needs to match in order to have an accurate calibration
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, ReferenceFrequency, 0); //initialize the VFO
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLB);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);// SET For Max Power
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA); // Set for reduced power for RX 
  
 
  TCCR1A = 0x00;
  TCCR1B = 0x01; // Timer1 Timer 16 MHz
  TCCR1B = 0x81; // Timer1 Input Capture Noise Canceller
  ACSR |= (1<<ACIC);  // Analog Comparator Capture Input
  
  pinMode(7, INPUT); //PD7 = AN1 = HiZ, PD6 = AN0 = 0
  digitalWrite(RX,LOW);         
}
 
//**************************[ END OF SETUP FUNCTION ]************************

//***************************[ Main LOOP Function ]**************************
void loop()
{  
  Band_Select();

  //--------------------------- LIMIT OUT OF OPERATION BANDS TO ONLY RX -----------------
  unsigned long freqdiv = freq / 1000000;
  if (freqdiv < 11 || freqdiv > 30){
    goto RX1; // Skip TX Loop
  }  

// --------------------------- FSK  TX LOOP -----------------------------------
 // The following code is from JE1RAV https://github.com/je1rav/QP-7C
  //(Using 3 cycles for timer sampling to improve the precision of frequency measurements)
  //(Against overflow in low frequency measurements)
  
  int FSK = 1;
  int FSKtx = 0;

  while (FSK>0){
    int Nsignal = 10;
    int Ncycle01 = 0;
    int Ncycle12 = 0;
    int Ncycle23 = 0;
    int Ncycle34 = 0;
    unsigned int d1=1,d2=2,d3=3,d4=4;
  
    TCNT1 = 0;  
    while (ACSR &(1<<ACO)){
      if (TIFR1&(1<<TOV1)) {
        Nsignal--;
        TIFR1 = _BV(TOV1);
        if (Nsignal <= 0) {break;}
      }
    }
    while ((ACSR &(1<<ACO))==0){
      if (TIFR1&(1<<TOV1)) {
        Nsignal--;
        TIFR1 = _BV(TOV1);
        if (Nsignal <= 0) {break;}
      }
    }
    if (Nsignal <= 0) {break;}
    TCNT1 = 0;
    while (ACSR &(1<<ACO)){
        if (TIFR1&(1<<TOV1)) {
        Ncycle01++;
        TIFR1 = _BV(TOV1);
        if (Ncycle01 >= 2) {break;}
      }
    }
    d1 = ICR1;  
    while ((ACSR &(1<<ACO))==0){
      if (TIFR1&(1<<TOV1)) {
        Ncycle12++;
        TIFR1 = _BV(TOV1);
        if (Ncycle12 >= 3) {break;}      
      }
    } 
    while (ACSR &(1<<ACO)){
      if (TIFR1&(1<<TOV1)) {
        Ncycle12++;
        TIFR1 = _BV(TOV1);
        if (Ncycle12 >= 6) {break;}
      }
    }
    d2 = ICR1;
    while ((ACSR &(1<<ACO))==0){
      if (TIFR1&(1<<TOV1)) {
        Ncycle23++;
        TIFR1 = _BV(TOV1);
        if (Ncycle23 >= 3) {break;}
      }
    } 
    while (ACSR &(1<<ACO)){
      if (TIFR1&(1<<TOV1)) {
      Ncycle23++;
      TIFR1 = _BV(TOV1);
      if (Ncycle23 >= 6) {break;}
      }
    } 
    d3 = ICR1;
    while ((ACSR &(1<<ACO))==0){
      if (TIFR1&(1<<TOV1)) {
        Ncycle34++;
        TIFR1 = _BV(TOV1);
        if (Ncycle34 >= 3) {break;}
      }
    } 
    while (ACSR &(1<<ACO)){
      if (TIFR1&(1<<TOV1)) {
        Ncycle34++;
        TIFR1 = _BV(TOV1);
        if (Ncycle34 >= 6) {break;}
      }
    } 
    d4 = ICR1;
    unsigned long codefreq1 = 1600000000/(65536*Ncycle12+d2-d1);
    unsigned long codefreq2 = 1600000000/(65536*Ncycle23+d3-d2);
    unsigned long codefreq3 = 1600000000/(65536*Ncycle34+d4-d3);
    unsigned long codefreq = (codefreq1 + codefreq2 + codefreq3)/3;
    if (d3==d4) codefreq = 5000;     
    if ((codefreq < 310000) and  (codefreq >= 10000)) {
       
        
      if (FSKtx == 0){
              
        //TX_State = 1;
        digitalWrite(RX,LOW);
        digitalWrite(TXSW,HIGH);
        digitalWrite(TX,HIGH);
         
        delay(10);
        
        si5351.output_enable(SI5351_CLK1, 0);   //RX off
        si5351.output_enable(SI5351_CLK0, 1);   //TX on
      }
    
      si5351.set_freq((freq * 100 + codefreq), SI5351_CLK0);
         
      FSKtx = 1;
    }
    else{
      FSK--;
    }
  }

 RX1:
  if(Osc_State != 0 || FSKtx != 0) //Do NOT update when oscillator state hasn't changed or not comming from a TX cycle
  {
    Osc_State = 0;
    FSKtx = 0;
    si5351.output_enable(SI5351_CLK0, 0);   //TX off
    digitalWrite(TX,0);
    digitalWrite(TXSW,0);

    si5351.output_enable(SI5351_CLK1, 0);   //RX off
    delay(10);
    rx_set_freq();
    delay(10);
    si5351.output_enable(SI5351_CLK1, 1);   //RX on

  /*
  Serial.println(freq);
  Serial.println(freq4); 
    */

    //TX_State = 0;
    digitalWrite(RX,HIGH);
  }
    
}
//*********************[ END OF MAIN LOOP FUNCTION ]*************************

//********************************************************************************
//******************************** [ FUNCTIONS ] *********************************
//********************************************************************************

//***************[ SI5351 oscillator set frequency function ]********************

bool rx_set_freq(){
  unsigned long freq4 = freq * 4;
  unsigned int freqdiv = freq / 1000000;

  if (freqdiv <= 25){ 
    si5351.set_freq(freq4*100ULL, SI5351_CLK1);
  } else {
    // For frequencies > 100 MHz
//  unsigned long long pllfreq = 70000000000ULL;
//  unsigned long long pllfreq = 90000000000ULL;
    unsigned long long pllfreq = freq4*100ULL*6; //6 * output frequency for 25 - 30 MHz means a PLL frequency between 600 and 720 MHz)
    si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
    si5351.set_freq_manual(freq4*100ULL, pllfreq , SI5351_CLK1);
    si5351.pll_reset(SI5351_PLLB);
  }
}

//***************[ Mode and Band Switch Frequency Select Function ]********************

void Band_Select(){
  unsigned int M = 0, B = 0, Mprev = 0, Bprev = 0;
  M = digitalRead(M_SW);
  B = digitalRead(B_SW);
  do { //Software debounce
    Mprev = M;
    Bprev = B;
    delay(100);
    M = digitalRead(M_SW);
    B = digitalRead(B_SW);
  } while((M != Mprev) || (B != Bprev));

  //Calculate button state
  unsigned int btns = 0;
  if ((B == LOW)&&(M == LOW)) btns = 1;
  else if ((B == LOW)&&(M == HIGH)) btns = 2;
  else if ((B == HIGH)&&(M == LOW)) btns = 3;
  else if ((B == HIGH)&&(M == HIGH)) btns = 4;

  //Only continue if button state really has changed
  if(btns == prevBtns) return;

  do { //Do-while loop for breaking purposes
    //Enter extended interface?
    if(prevTime == 0) {
      //Check for simultanious switch of band and mode
      if((btns == 1 && prevBtns == 4) || (btns == 4 && prevBtns == 1) || (btns == 2 && prevBtns == 3) || (btns == 3 && prevBtns == 2)) {
        prevBtns = btns;
        //Entering extended interface

        //Indicate state change
        Blink_TX_Led_Short(2);
        delay(500);
        Blink_TX_Led_Extra_Long();
        Blink_TX_Led_Extra_Long();
        if(bandPushCount != 0) {
          Blink_TX_Led_Short(bandPushCount);
          delay(500);
        }
        Blink_TX_Led_Extra_Long();
        Blink_TX_Led_Short(modePushCount);

        //Entered extended interface. We are done.
        prevTime = millis();
        return;
      }

      //Fall through to normal interface mode
      break;
    }

    //Handle extended interface states:

    //Check for simultanious switch of band and mode
    if((btns == 1 && prevBtns == 4) || (btns == 4 && prevBtns == 1) || (btns == 2 && prevBtns == 3) || (btns == 3 && prevBtns == 2)) {
      //Exit extended interface mode
      prevTime = 0;
      //Fall through to normal interface mode
      break;
    }

    //Check for time expired since last extended interface state change
    unsigned long time = millis();
    if(time - prevTime > 5000) {
      prevTime = 0;
      //Fall through to normal interface mode
      break;
    }
    prevTime = time;

    //Check for band change
    if((btns == 1 && prevBtns == 3) || (btns == 3 && prevBtns == 1) || (btns == 2 && prevBtns == 4) || (btns == 4 && prevBtns == 2))
    {
      prevBtns = btns;
      bandPushCount++;
      if(bandPushCount > 5) bandPushCount = 1;

      if(Freq_assign_extended()) {
        //Indicate push count
        Blink_TX_Led_Extra_Long();
        Blink_TX_Led_Short(bandPushCount);
      } else {
        //Error
        Blink_TX_Led_Long(3);
        break;
      }

      //Band changed. We are done.
      prevTime = millis();
      return;
    }

    //Check for mode change
    if((btns == 1 && prevBtns == 2) || (btns == 2 && prevBtns == 1) || (btns == 3 && prevBtns == 4) || (btns == 4 && prevBtns == 3))
    {
      prevBtns = btns;
      modePushCount++;
      if(modePushCount > 6) modePushCount = 1;

      if(Freq_assign_extended()) {
        //Indicate push count
        Blink_TX_Led_Extra_Long();
        Blink_TX_Led_Short(modePushCount);
      } else {
        //Error
        Blink_TX_Led_Long(3);
        break;
      }

      //Mode changed. We are done.
      prevTime = millis();
      return;
    }

  } while(1);

  //Normal interface
  prevBtns = btns;
  switch(btns) {
    case 1:
      if(freq != B1_FT8) {
        freq = B1_FT8;
        Osc_State = 1;
      }
      break;
    case 2:
      if(freq != B1_FT4) {
        freq = B1_FT4;
        Osc_State = 1;
      }
      break;
    case 3:
      if(freq != B2_FT8) {
        freq = B2_FT8;
        Osc_State = 1;
      }
      break;
    case 4:
      if(freq != B2_FT4) {
        freq = B2_FT4;
        Osc_State = 1;
      }
      break;
  }

  //Indicate state change
  Blink_TX_Led_Short(1);
}

void Blink_TX_Led_Short(unsigned int blinks) {
    for(unsigned int i = 0; i < blinks; i++) {
      digitalWrite(TX, HIGH);  
      delay(50);
      digitalWrite(TX, LOW);
      if(i != blinks-1) delay(350);
    }
}

void Blink_TX_Led_Long(unsigned int blinks) {
    for(unsigned int i = 0; i < blinks; i++) {
      digitalWrite(TX, HIGH);  
      delay(200);
      digitalWrite(TX, LOW);
      if(i != blinks-1) delay(200);
    }
}

void Blink_TX_Led_Extra_Long() {
    digitalWrite(TX, HIGH);  
    delay(500);
    digitalWrite(TX, LOW);
    delay(500);
}

//***************[ Band Switch Frequency and Mode Assign Function ]********************

void SW_assign(){
    Freq_assign(BAND1);
    B1_FT8 = F_FT8;
    B1_FT4 = F_FT4;

    Freq_assign(BAND2);
    B2_FT8 = F_FT8;
    B2_FT4 = F_FT4;
}

//*********************[ Band dependent Frequency Assign Function ]********************
void Freq_assign(unsigned int Band) {

  switch (Band) {
   
    case 20:
      F_FT8 = 14074000;
      F_FT4 = 14080000;
      F_FT2 = 14084000;
      F_JT9 = 14078000;
      F_JT65 = 14076000;
      F_WSPR = 14095600;
      break;
    case 17:
      F_FT8 = 18100000;
      F_FT4 = 18104000;
      F_FT2 = 18108000;
      F_JT9 = 18104000;
      F_JT65 = 18102000;
      F_WSPR = 18104600;
      break;
    case 15:
      F_FT8 = 21074000;
      F_FT4 = 21140000;
      F_FT2 = 21144000;
      F_JT9 = 21078000;
      F_JT65 = 21076000;
      F_WSPR = 21094600;
      break;
    case 12:
      F_FT8 = 24915000;
      F_FT4 = 24919000;
      F_FT2 = 24923000;
      F_JT9 = 24919000;
      F_JT65 = 24917000;
      F_WSPR = 24924600;
      break;
    case 10:
      F_FT8 = 28074000;
      F_FT4 = 28180000;
      F_FT2 = 28184000;
      F_JT9 = 28078000;
      F_JT65 = 28076000;
      F_WSPR = 28124600;
      break;
  }
}

bool Freq_assign_extended() {
  //Sanity check
  if(bandPushCount > 5 || modePushCount > 6) return false;

  //Switch band accordingly
  unsigned int b = 20;
  switch(bandPushCount) {
    case 0: {
      unsigned long freqdiv = freq / 1000000;
      if(freqdiv == (B1_FT8 / 1000000)) b = BAND1;
      if(freqdiv == (B2_FT8 / 1000000)) b = BAND2;
      break;
    }
    case 1:
      b = 10;
      break;
    case 2:
      b = 12;
      break;
    case 3:
      b = 15;
      break;
    case 4:
      b = 17;
      break;
  }
  Freq_assign(b);
  switch(modePushCount) {
    case 0:
      if(freq == F_FT4 || freq == B1_FT4 || freq == B2_FT4) freq = F_FT4;
      else freq = F_FT8;
      break;
    case 1:
      freq = F_FT8;
      break;
    case 2:
      freq = F_FT4;
      break;
    case 3:
      freq = F_FT2;
      break;
    case 4:
      freq = F_JT9;
      break;        
    case 5:
      freq = F_JT65;
      break;        
    case 6:
      freq = F_WSPR;
      break;        
  }
  Osc_State = 1;

  return true;
}
//************************[ End of Frequency assign function ]*************************
