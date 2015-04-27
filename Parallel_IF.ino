/*
Parallel_IF

Demonstration of the Parallel IF architecture
on a simple BITX superhet 
with Si5351
with Rotary Encoder under Interrupt Interface
and support for 16*4 or 20*4 LCD

As Demonstrated at FDIM, 2015

Developed from 
BITX_VERO_0v1

m0xpd
=======================================================
*/
 
#include<stdlib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Rotary.h>
#include "si5351.h" 
/*=====================================================
Frequency defaults...
 
The system is set up to use the 
80m, 40m, 20m, 17m amateur bands 
*/ 
double BandBases[]={
  3500000, 7000000, 14000000, 18068000}; 
 double BandTops[]={
  4000000, 7200000, 14350000, 18168000};
double BandCWFreqs[]={
  3700000, 7090000, 14285000, 18130000}; 
  
/*=====================================================
Modes and Offsets...
 
The system offers the following "modes"...
LSB, USB, which change either side of 10MHz
each of which has its own receive offset
 
You can adjust the receive offset continuously 
(to achieve "Receive Incremental Tuning" or "Clarifier" operation)
and you can change the offset defaults by editing the lines below... */
 
int ModeOffsets[]={
  0, 0};
/*=====================================================
Intermediate Frequency Settings...

The system implements BFO settings for lower and upper sideband
The BFO is changed when the band changes (if required)
*/
double BFOs[][2]={
{10000600, 9998450},   
{11996700, 11995800},  
};  
int  CWShift=400; 
 
/*======================================================================
Defining CONSTANTS...
*/
//======================================
// HARDWARE: Arduino I/O Pin allocations..
// set pin numbers:
const int DATA = 8;
// input from rig to signal Tx/Rx status
const int TxPin = 12;
// input to rig to turn on CW
const int TxKey = 13;
// Rotary Encoder...
const int RotEncAPin = 2;
const int RotEncBPin = 3;
const int RotEncSwPin = A3;
// Pushbuttons...
const int modeSw1 = 4;
const int modeSw3 = 5;
// CW Key / Paddle inputs...
const int str8Pin = A0;
const int ditPin = A1;  
const int dahPin = A2;
// Display...
// the display uses the I2C connection, 
// which uses 
// A4 for the Clock and
// A5 for the Data
//
int lcdWidth=16;
int L2=17;
int L3=1;

//======================================
// SOFTWARE: defining other constants...
unsigned sidetoneFreq=1800;           // the frequency of the SideTone buzz!
int Tx_hold = 8000;                  // count for Break-in: reduce to move toward full QSK
int Tx_count = 0;              // count variable

/* MENU STRUCTURE....

MENU 0: RiT        Scalar
MENU 1: VFO        A or B
MENU 2: SideBand   USB or LSB
MENU 3: Band       80, 40, 20, or 17
MENU 4: IF         10 or 8 
MENU 5: ALIGNMENT  Scalar
*/

const int nMenus = 0x05;
byte nMenuOptions[] = {
  1, 1, 1, 3, 1, 1};
char* MenuString[6][12]= {
  {
    "RiT:   ", "       " }
  ,
  {
    "VFO A   ", "VFO B   " }
  ,
  {
    "LSB    ", "USB     "}
  ,
  {
    "  80m  " ,"  40m  ", "  20m  ", "  17m  "}
  ,
  {
    "SSB     ", "CW      "} 
  ,
  {
    "BFO:     ", "         " } 
};

byte BandSwitch[]={0x01, 0x02, 0x04, 0x08};


// scaling factors for freq adjustment
const long deltaf[] = {
  1, 10, 100, 1000, 10000, 100000, 0, 1000000};
// Marketing !
char* Banner="   Parallel IF  ";
// end of Constants
//=================================================================
 
/*=================================================================
Declare (and initialize) some VARIABLES...
*/

// Iambic Keyer parameters...
int ditLength = 75;            // dit length in milliseconds - ADJUST SPEED HERE!
int dahLength = 3 * ditLength;
int ditState = 0;              // variables for reading the key status
int dahState = 0;
int str8State = 0;
int LastElement = 0;            // code for the last element sent...
                                // 0 :- space
                                // 1 :- dit
                                // 2 :- dah 
boolean MenuMode = false;
boolean Transmit=false;
boolean TransmitCW=false;
char* TxChar;
int OffsetSign=0; 
char OffsetChar='=';
char VFOChar='A';
byte MenuOption[] = {
  0, 0, 0, 1, 0, 0};
byte oldVFO=MenuOption[1];
int RiT=ModeOffsets[MenuOption[2]];
double freq = BandCWFreqs[MenuOption[3]];
double freqA=freq;
double freqB=freq;
double BFO;
int dfindex = 3;
// Declare and initialise some Rotary Encoder Variables...
boolean OldRotEncA = true;
boolean OldRotEncB = true;
boolean RotEncA = true;
boolean RotEncB = true;
boolean RotEncSw = true;
 
boolean modeSw1State = HIGH;
boolean modeSw3State = HIGH;
boolean TxPinState = HIGH;

// Display positions...
byte ModePos=16;
byte TxPos=20;
byte RiTSignPos=23;
byte VFOPos=26;
byte BandPos=28;
int BannerPos=0;

// some temporary cursor variables for LCD management...
int c1=0;
int c2=0;
int c3=0;

int Turns = 0;

int Menu = 0;
 
// end of Variables
//=================================================================

// Instantiate the LCD display...
LiquidCrystal_I2C lcd(0x20,32,2); 

// Instantiate the Si5351...
Si5351 si5351;

// Instantiate the Rotary Encoder...
Rotary r = Rotary(RotEncBPin, RotEncAPin);

/**************************************/
/* Interrupt service routine for      */
/* encoder frequency change           */
/**************************************/
ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_CW)
    Turns=Turns+1;
  else if (result == DIR_CCW) 
    Turns=Turns-1;
}


void setup() {
  lcd.init(); 
  delay(200);
  lcd.init();  
  lcd.blink();

 
  // manage LCD width:
  if (lcdWidth==20){
     lcd.backlight();
     ModePos=20;
     TxPos=25;
     RiTSignPos=30;
     VFOPos=34;
     BandPos=35;
     L2=23; 
     L3=1;
     BannerPos=2;    
  } 
 
  
  // initialize the Key pins as inputs:
  pinMode(ditPin, INPUT);
  pinMode(dahPin, INPUT);
  pinMode(str8Pin, INPUT); 
  pinMode(modeSw1,INPUT);
  pinMode(modeSw3,INPUT);  
  pinMode(TxPin,INPUT);  
  pinMode(TxKey,OUTPUT);  
  pinMode(RotEncSwPin, INPUT);
  // set up pull-up resistors on inputs...  
  digitalWrite(ditPin,HIGH);  
  digitalWrite(dahPin,HIGH);
  digitalWrite(str8Pin,HIGH); 
  digitalWrite(modeSw1,HIGH);
  digitalWrite(modeSw3,HIGH); 
  digitalWrite(RotEncSwPin,HIGH); 
  digitalWrite(TxPin,HIGH);
  // Print opening message to the LCD...
  Normal_Display();
  si5351.init(SI5351_CRYSTAL_LOAD_8PF);  
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  
  Serial.begin(9600);

  // start the oscillators...
  sendFrequency(freq);    
  BFO=BFOs[0][MenuOption[2]];
  sendbfo(BFO);
  // why do I need a second call to the vfo ???
  sendFrequency(freq);  
  // Enable pin change interrupt for the encoder
  PCICR |= (1 << PCIE2);                                                
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  
  
  Serial.println(BFO);   

}
 
void loop() {
  // First, we check for SSB Transmit...
   TxPinState = digitalRead(TxPin);
  if (TxPinState==0){
    if  (Transmit==false){
      Transmit=true;
      setTxDisplay();
    }
  }
  else{
  // No :- we're in "Receive" 
   if (Transmit==true){
    Transmit=false;
    setTxDisplay();
  }
  
  }
  // Read the inputs...
  ditState = digitalRead(ditPin);
  dahState = digitalRead(dahPin);
  str8State = digitalRead(str8Pin);
  RotEncSw = digitalRead(RotEncSwPin);
  modeSw1State=digitalRead(modeSw1);
  modeSw3State=digitalRead(modeSw3);
  
  
  if (MenuMode){
    // Here we're in "Menu" mode...
    if (Menu==0){
    if (Turns!=0){
    RiT=constrain(RiT+Turns,-100000,10000);
    Turns=0; 
    LCD_Display_RiT(RiT);    
    sendFrequency(freq); 
    if (RiT>ModeOffsets[MenuOption[2]]) {
    OffsetSign=1;
    } 
    else{
    if (RiT==ModeOffsets[MenuOption[2]]){
    OffsetSign=0;
    }
    else{
    OffsetSign=-1;
    }
    } 
    } 
   
   
    } 


    if (Menu==5){
    if (Turns!=0){
    BFOs[MenuOption[4]][MenuOption[2]]=constrain(BFOs[MenuOption[4]][MenuOption[2]]+Turns*50,1000000,15000000);
    Turns=0;
    BFO=BFOs[MenuOption[4]][MenuOption[2]];
    sendbfo(BFO);
    sendFrequency(freq);  // update the VFO too 7 Feb 2015
    LCD_Display_BFO(BFO);      
    }  

    }



    // we're not in menu 0 or 5, so manage the menu options 
    // for all other menus...
    if ((Turns!=0)&&(Menu!=0)&&(Menu!=5)){
     MenuOption[Menu]=constrain(MenuOption[Menu]+Turns,0,nMenuOptions[Menu]);
     Turns=0;
     LCD_String_Display(7, 0, (MenuString[Menu][MenuOption[Menu]]));
    } 

 
    if (modeSw1State==LOW){
      // decrement Menu
      Menu=constrain(Menu-1,0,nMenus);
      if(Menu==2){
        // skip the mode change for the moment
        Menu = 1;
      }     
      LCD_Int_Display(5, 0, Menu);
      LCD_String_Display(7, 0, (MenuString[Menu][MenuOption[Menu]]));
      delay(500);
    } 
 
    if (modeSw3State==LOW){
      // Increment Menu
      Menu=constrain(Menu+1,0,nMenus);
      // Pete needs the ability to switch USB or LSB - so keep menu 2...
//       if(Menu==2){
//        // skip the mode change for the moment
//        //Menu = 3;
//      }      
      LCD_Int_Display(5, 0, Menu); 
      LCD_String_Display(7, 0, (MenuString[Menu][MenuOption[Menu]]));     
      delay(500);
 
    }   
  } 
// End of MenuMode==1
// ========================================================================
// Beginning of normal mode (MenuMode==0)...
else{
    // Straight key has top priority
  if (str8State == LOW) {              // if straight key is pressed...

    StartCount();                      //   start the break-in count
    tone(DATA,sidetoneFreq);    //   turn on the SideTone
  }
  else {
    noTone(DATA);              //   and SideTone
  }  
  
// Secondly, deal with the paddle...
// The Occam's Microcontroller Iambic Keyer follows the same logical structure I've published before:
//   here for Arduino: http://m0xpd.blogspot.co.uk/2013/02/keyerduino.html
//   here for Raspberry Pi: http://m0xpd.blogspot.co.uk/2012/12/keyer-on-rpi.html
//   and here for PIC: http://m0xpd.blogspot.co.uk/2009/11/m0xpd-keyer.html
//
  if ((ditState == LOW)&(dahState == HIGH)) {        // dit is pressed    
    LastElement = 1;                                 //   remember that
    CWKey(ditLength);                                //   key the Tx for a dit
    CWSpace(ditLength);                              //   then a space
  } 
  else if ((dahState == LOW)&(ditState == HIGH)) {   // dah is pressed 
    LastElement = 2;                                 //   remember that
    CWKey(dahLength);                                //   key the Tx for a dah
    CWSpace(ditLength);                              //   then a space   
  }  
  else if ((dahState == LOW)&(ditState == LOW)) {    // both dit and dah are pressed
    if (LastElement == 2) {                          //   if last element was a dah
      LastElement = 1;                               //   Remember that this will be a dit
      CWKey(ditLength);                              //   and send it
      CWSpace(ditLength);
    }
    else {                                           //   last element was a dit
      LastElement = 2;                               //   so remember this will be a dah               
      CWKey(dahLength);                              //   and send it
      CWSpace(ditLength);
    }    
  }
  else {
    LastElement = 0;                                  // no paddle activity this pass
  }
// End of dealing with key and paddle
  
  
  

  
  if (Turns!=0){
    // adjust frequency
    freq=constrain(freq+constrain(Turns,-5,5)*deltaf[dfindex],BandBases[MenuOption[3]],BandTops[MenuOption[3]]);
    Turns=0;
    LCD_Display_Freq(freq);
    sendFrequency(freq);         
  }  
  
      if (modeSw1State==LOW){
      // point to a higher digit
      dfindex=constrain(dfindex+1,0,7);
      LCD_Display_Freq(freq);
      delay(500);
 
    } 
 
    if (modeSw3State==LOW){
      // point to a lower digit
      dfindex=constrain(dfindex-1,0,7);
      LCD_Display_Freq(freq);       
      delay(500);
 
    }  
    // End of Normal Mode
}
 
  if (RotEncSw==LOW){
  // Toggle in and out of menu mode
    if (MenuMode == false){
      // enter menu mode
      MenuMode = true;
      LCD_String_Display(0, 0, "Menu         ");
      LCD_Int_Display(5, 0, Menu);
      LCD_String_Display(7, 0, (MenuString[Menu][MenuOption[Menu]]));
    }
    else{
      // leave menu mode
      MenuMode = false;
      //====================
      // specific actions on leaving menus...
 
  switch (Menu){
  // Leaving RiT adjustment
  case 0:
    switch (OffsetSign) {
      case 1:
      OffsetChar='+';
      break;
      case -1:
      OffsetChar='-';
      break;
      default: 
      OffsetChar='=';
    } 
 
  break;  
  // Leaving VFO swap
  case 1:
  if(oldVFO==0){
  freqA=freq;
  if (MenuOption[1]==1){
  freq=freqB;
  } 
  }
  else{
  freqB=freq;
  if (MenuOption[1]==0){
  freq=freqA;
  }
  }
  oldVFO=MenuOption[1];
  break;
  case 2:
  // leaving mode change
  RiT=ModeOffsets[MenuOption[2]];
  OffsetChar='=';
  break;
  case 3:
  // Leaving Band Change
    freq=BandCWFreqs[MenuOption[3]];
    freqA=freq;
    freqB=freq;
    MenuOption[1]=0;
    if(MenuOption[3]>1){
    //put into USB mode...
    MenuOption[2]=1;
    }
    else{
   // put into LSB
    MenuOption[2]=0;
    }
    // Now set the GPIO Expander outputs...
//    Wire.beginTransmission(0x27);
//    Wire.write(0x09);                         // Port A
//    Wire.write(BandSwitch[MenuOption[3]]);    // data
//    Wire.endTransmission();
  }
    //====================
    // 
        BFO=BFOs[MenuOption[4]][MenuOption[2]];
      sendbfo(BFO);
     Serial.println(BFO);   
    sendFrequency(freq);
    Normal_Display();

    } 
  
    delay(500);
// End of Toggle In and Out of Menu Mode
  }    
    OldRotEncA=RotEncA; 
// End of Receive Mode
// Now we do housekeeping at the end of the main loop...  
// Decrement the Transmit Count
  Tx_count = constrain(Tx_count-1,0,Tx_hold);
  if ((Tx_count == 0)&&(TransmitCW==true)) {  // are we at the end of the transmit phase?
    digitalWrite(TxKey,LOW);                // YES - so un-mute the Rx
    TransmitCW = false;
          BFO=BFOs[MenuOption[4]][MenuOption[2]];
      sendbfo(BFO);
    sendFrequency(freq);
      setTxDisplay();
  }
// End of loop()
}
 
 
//==============================================================
// SUBROUTINES...
 
// subroutine to display the frequency...
void LCD_Display_Freq_2(double frequency) {  
  lcd.setCursor(17, 0);
  if (frequency<10000000){
  lcd.print(" ");
  }  
  lcd.print(frequency/1e6,6);  
  lcd.print(" MHz");
  // establish the cursor position
  int c_position=L2+8-dfindex;
  lcd.setCursor(c_position, 0);
  //lcd.blink();   
}  


// new subroutine to display the frequency...
void LCD_Display_Freq(double frequency) {  

  // first deal with the high frequency stuff above 10 MHz
  if (frequency>10000000){
  lcd.setCursor(L2+1,0);    
  lcd.print(frequency,0);  
  lcd.print(" MHz ");
  lcd.setCursor(L2,0);
  lcd.print(floor(frequency/1e6),0);  
  lcd.print(".");  
  }  
  else{  // handle the LF range below 1 MHz
  if(frequency<1000000){
  lcd.setCursor(L2+3,0);    
  lcd.print(frequency,0);  
  lcd.print(" MHz ");
  lcd.setCursor(L2+1,0);  
  lcd.print("0.");  
  }
  else{  // this is the normal range, between 1 and 10 MHz
  lcd.setCursor(L2, 0);
  lcd.print("  ");
  lcd.print(frequency,0);
  lcd.print(" MHz ");
  lcd.setCursor(L2+1,0);
  lcd.print(floor(frequency/1e6),0);  
  lcd.print(".");
  }
  }  
  // establish the cursor position
  int c_position=L2+8-dfindex;
  lcd.setCursor(c_position, 0); 
} 

// subroutine to display the BFO...
void LCD_Display_BFO(double BFO) {
  lcd.setCursor(L3+1,1);
  lcd.print(BFO,1);  
  lcd.print(" Hz ");
} 

 
 
// subroutine to display the RiT...
void LCD_Display_RiT(int RiT) {
  lcd.setCursor(5, 1);
  lcd.print(RiT,DEC);  
  lcd.print(" Hz   "); 
}  
 
// subroutine to clear the RiT display...
void LCD_Clear_RiT() {
  lcd.setCursor(0, 1);
  if(lcdWidth==16){
  lcd.print("               "); 
  }
  else{
  lcd.print("                   ");     
  } 
} 
 
// subroutine to display a string at a specified position...
void LCD_String_Display(int c1, int c2, char Str[ ]){
      lcd.setCursor(c1, c2);
      lcd.print(Str);
}
// subroutine to display a number at a specified position...
void LCD_Int_Display(int c1, int c2, int num){
      lcd.setCursor(c1, c2);
      lcd.print(num);
}
// subroutine to display a string at a specified position...
void LCD_Char_Display(int c1, int c2, char Str){
      lcd.setCursor(c1, c2);
      lcd.print(Str);
}
 
 
// subroutine to set up the normal display...
void Normal_Display(){
      LCD_String_Display(ModePos, 1, MenuString[2][MenuOption[2]]);
      LCD_String_Display(BandPos, 1, MenuString[3][MenuOption[3]]);     
      switch (MenuOption[1]) {
        case 0:
          VFOChar='A';
        break;
        case 1:
          VFOChar='B';
        break;
        default: 
        VFOChar=' ';    
       }  
     LCD_Char_Display(VFOPos, 1, VFOChar);      
     LCD_Clear_RiT(); 
     LCD_String_Display(BannerPos, 0, Banner);     
     setTxDisplay();
     LCD_Char_Display(RiTSignPos, 1, OffsetChar);
     LCD_Display_Freq(freq);
}
 
// Subrouting to display the Transmit Status
void setTxDisplay(){
      switch (Transmit) {
        case false:
        TxChar="Rx";
        break;
        case true:
        TxChar="Tx";
        break; 
       }  
      LCD_String_Display(TxPos, 1,TxChar); 
      // re-establish the cursor position
      int c_position=L2+8-dfindex;
      lcd.setCursor(c_position, 0);
}
 
// Subroutine to generate a positive pulse on 'pin'...
void pulseHigh(int pin) {
digitalWrite(pin, HIGH); 
digitalWrite(pin, LOW); 
}
 
// calculate and send frequency code to DDS Module...
void sendFrequency(double frequency) {
  double freq1=0;
  if(MenuOption[2]==0){
  freq1=BFOs[MenuOption[4]][0]-frequency;//+MenuOption[4]*100;  
  }
  else{
  //freq1=frequency-MenuOption[4]*100-BFOs[MenuOption[4]][1];
  freq1=frequency-BFOs[MenuOption[4]][1];
  }
  if(Transmit==false){
  freq1=freq1-RiT;
  }
  if(TransmitCW==true){
  freq1=freq1-1200;
  }
  //vfo.setFrequency(freq1);
  si5351.set_freq(freq1, 0, SI5351_CLK1);
}

// calculate and send frequency code to DDS Module...
void sendCWTxFrequency(double frequency) {
  double freq1=0;
  if(MenuOption[2]==0){
  freq1=BFOs[0][0]+CWShift-frequency;
  sendbfo(BFOs[0][0]+CWShift);
  }
  else{
  freq1=frequency-BFOs[0][1]-CWShift;
  sendbfo(BFOs[0][1]-CWShift); 
  }
  if(Transmit==false){
  freq1=freq1-RiT;
  }
  if(TransmitCW==true){
  if(MenuOption[2]==0){  
  freq1=freq1-1200;
  }
  else{
  freq1=freq1-1200+2*CWShift;
  }
  }
  //vfo.setFrequency(freq1);
  si5351.set_freq(freq1, 0, SI5351_CLK1);
}


void sendbfo(double frequency) {
  //bfo.setFrequency(frequency);
  // CLK0 is the BFO
  si5351.set_freq(frequency, SI5351_PLL_FIXED, SI5351_CLK0);
}

// Subroutine to Key the Tx for a dit OR a dah
void  CWKey(int duration){
  StartCount();                      // start the Tx count
  tone(DATA,sidetoneFreq);    // start the sidetone
  delay(duration);                   // wait for the element duration
  noTone(DATA);               // stop the SideTone
}

// Subroutine to start a transmit phase
// (and mute the Rx)
void StartCount() {
    if(TransmitCW==false){
    TransmitCW=true;  
    sendCWTxFrequency(freq); 
    }
    digitalWrite(TxKey,HIGH);          //   turn on the Tx
    Tx_count=Tx_hold;                            // initialise the Tx_Count
    //
  // digitalWrite(RxMute,LOW);                   // mute the Rx 
}

// Subroutine to wait for an inter-element space
void  CWSpace(int duration){
  delay(duration);
}
