#include <SPI.h>
// #include "TMC4361A_Register.h"
#include <JC_Button.h>
 
Button button1(7,25);
Button button2(5,25);
Button button3(4,25);
Button button4(3,25);
int lockpin = 6;
int chipCS = 10;
const byte CLOCKOUT = 9;
int enable = 8;
int nFreeze = 2;
int32_t pos,steps=0;
bool lock = false;

void setup()
{
 pinMode(chipCS,OUTPUT);
 pinMode(CLOCKOUT,OUTPUT);
 pinMode(enable, OUTPUT);
 pinMode(nFreeze, OUTPUT);
 
 digitalWrite(chipCS,HIGH);
 digitalWrite(enable,LOW);
 digitalWrite(nFreeze,HIGH);
 
 button1.begin();
 button2.begin();
 button3.begin();
 button4.begin();
 pinMode(lockpin,INPUT);

//  set up Timer1
 TCCR1A = bit (COM1A0); //toggle OC1A on Compare Match
 TCCR1B = bit (WGM12) | bit (CS10); //CTC, no prescaling
 OCR1A = 0; //output every cycle
 
// PLLFRQ |= _BV(PLLTM0); // use 96MHz PLL clock for Timer 4
// TCCR4B = _BV(CS40) + _BV(PWM4X); // PCK/1 in asynchronous mode
// TCCR4D = 0; // WGM41/40 = 0, Fast PWM, OCR4C = TOP
// OCR4C = 0x5; // TOP value: 96MHz / 6 = 16MHz
// TCCR4A = _BV(COM4B0) // /OC4B connected, cleared on compare match, set when TCNT4 = 0,
// + _BV(PWM4B); // enable PWM based on OCR4B
// OCR4B = 2; // 50% duty cycle

 SPI.setBitOrder(MSBFIRST);
 SPI.setClockDivider(SPI_CLOCK_DIV8);
 SPI.setDataMode(SPI_MODE3);
 SPI.begin();
 
 Serial.begin(9600);
 
 
 sendData(0xCF,0x52535400); //software reset of tmc4361

 sendData(0xB1,0x007A1200); //clock 8Mhz
//  sendData(0xB1,0x00F42400); //clock 16Mhz

//  sendData(0x80,0x00006020);
 sendData(0x80,0x00006026); // general_conf: direct-accl, direct-bow, diff enc enabled

//  sendData(0x90,0x00060004); // Steplength

 sendData(0x8A,0x00FB1900); //400steps/rev stepper

// sendData(0x83,0x00003300);
 sendData(0x83,0x00000022); // input filter for encoder pins



 sendData(0xD4,0x00004000); // Encoder resolution = 16384 pulse/rev
 sendData(0x87,0x20000400); //invert encoder

// SPI output interface is connected with a TMC26x/389 stepper motor driver. 
// Configuration and current data are transferred to the stepper motor driver.
//spi_out_low_time: 4, spi_out_high_time: 4, spi_out_block_time: 8, cover data length: 0(auto)
// Enabling of automatic continuous streaming of cover datagrams.
 sendData(0x84,0x8440008A); //spi_out_conf
 

 //    ************************Setting TMC2660 parameters***********************

 sendData(0x8D,0x02000000); //Assign COVERDONE as INTR

 sendData(0x0E,0x00000000);//Clear events
 
 sendData(0xEC,0x00000000);

 sendData(0x0E,0x00000000);//Clear events

 sendData(0xEC,0x00090585); //Set the CHOPCONF register of TMC26x/389 (cover datagram): 
 //tbl=36, standard chopper, HDEC=16, HEND=11, HSTR=1, TOFF=5, RNDTF=off

 sendData(0x0E,0x00000000);//Clear events

 sendData(0xEC,0x000A0000); //Disable the SMARTEN register of TMC26x/389 (cover datagram)

 sendData(0x0E,0x00000000);//Clear events




 sendData(0xEC,0x000C0014); //Set the SGCSCONF register of TMC26x/389 (cover datagram):
//SGT=0, CS=23




 sendData(0x0E,0x00000000);//Clear events

 sendData(0xEC,0x000EF080); //Set the DRVCONF register of TMC26x/389 (cover datagram): 
//SLPH=3, SLPL=3, DISS2G=off, TS2G=0-3.2us, SDOFF=on, VSENSE=0

 sendData(0x0E,0x00000000);//Clear events

//        **************************************************************


 sendData(0xA4,0x00000000); // v = 0
 sendData(0xA1,0x00000000); // xactual = 0
 sendData(0xB7,0x00000000); // xtarget = 0
 
 // ******************Closed Loop calibration of TMC4361 Motion Controller
 
 
 sendData(0x9C,0x00FF00FF); // CL_BETA = CL_GAMMA = 255 default value for best performance
 
 sendData(0xA0,0x00000004); // hold + position mode
 
 sendData(0xA4,0x00100000); // slow velocity
 
 sendData(0xDC,0x00010000); // cl_p = 1.0
 
 sendData(0x87,0x00400000); // turn on closed loop
 
 sendData(0xB7,0x00000080); // move to full step position
 
 sendData(0x87,0x01400000); // turn on closed loop calibration
 
 sendData(0x87,0x00400000); // turn off closed loop calibration
 
 sendData(0xA4,0x00000000); // v = 0
 
 
 // ***************************Setup Closed Loop Operation
 
 sendData(0xA0,0x00000005); // trapezoidal-Ramp + POS Mode

 sendData(0xA4,0x01400000); // VMAX 
 sendData(0xA8,0x00640000); // AMAX
 sendData(0xA9,0x00640000); // DMAX
 
//  sendData(0xAD,0x00000100); // bow1
//  sendData(0xAE,0x00000100); // bow2
//  sendData(0xAF,0x00000100); // bow3
//  sendData(0xB0,0x00000100); // bow4
 
//  sendData(0xE0,0x00250000); // emf_vmin = 
//  sendData(0xE1,0x00450000); // emf_vadd = -> emf_vmax = 
 
//  sendData(0xE2,0x00FFFFFF); // emf_vel0_timer
//  sendData(0xE3,0x02000864); // enc vel filter settings
 




 sendData(0xDF,0x00000014); // cl_tolerance = 20 Closed loop tolerance (microsteps)






//  sendData(0xDC,0x00010000); // cl_p = 1.0
 sendData(0xDC,0x00014000); // cl_p = 1.25?


 sendData(0xDA,0x000000C8); // cl_vlimit_p = 200
 sendData(0xDB,0x00000032); // cl_vlimit_i = 50
 sendData(0xDD,0x000003E8); // cl_vlimit_diclip = 1000
 sendData(0xDE,0x00100000); // cl_vlimit = 100.000 pps
 
 sendData(0x86,0x0032f064); // cl scaling values
 sendData(0x98,0x00001000); // cl_upscale
 sendData(0x99,0x00100000); // cl_dnscale
 
 sendData(0x87,0x28400000); // cl with gamma correction and vlimit
 sendData(0x85,0x00000080); // cl scaling on

}
 
void loop()
{
 button1.read();
 button2.read();
 button3.read();
 button4.read();
// pos=sendData(0x21,0);
//    steps=160448;
//     sendData(0xB7,pos+steps); Serial.println(sendData(0x21,0));
//  Serial.println(sendData(0x50,0));
//  delay(6000);
//  pos=sendData(0x21,0);
//    steps=-160448;
//     sendData(0xB7,pos+steps);
//  Serial.println(sendData(0x21,0));
//  Serial.println(sendData(0x50,0));
//  delay(6000);
 if (button1.wasReleased() && !lock){
   pos=sendData(0x21,0);
   steps=16448;
    sendData(0xB7,pos+steps); // XTARGET = 100.000   //steps moved for pressing button 1 
   }
  if (button2.wasReleased() && !lock){
    pos=sendData(0x21,0);
   steps=-16448;
    sendData(0xB7,pos+steps); // XTARGET = 100.000  //steps moved for pressing button 2 
   }
  if (button3.wasReleased() && !lock){
    pos=sendData(0x21,0);
   steps=118048;
    sendData(0xB7,pos+steps); // XTARGET = 100.000  //steps moved for pressing button 3
   }
  if (button4.wasReleased() && !lock){
    pos=sendData(0x21,0);
   steps=-118048;
    sendData(0xB7,pos+steps); // XTARGET = 100.000  //steps moved for pressing button 4
   }

   //lock button and indicator LED function
   //if (button5.wasReleased()){
    //attachInterrupt(0, wakeUp, LOW);
    if (digitalRead(lockpin)){
    //digitalWrite(ledPin, LOW);
    //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    lock = true;
   }
   else{
        //digitalWrite(ledPin, HIGH);
        lock = false;
   }
}
 
int32_t sendData(unsigned long address, unsigned long datagram)
{
 //TMC4361 takes 40 bit data: 8 address and 32 data
 
 delay(100);
 int32_t  i_datagram;
 
 digitalWrite(chipCS,LOW);
 delayMicroseconds(10);
 
 SPI.transfer(address);
 
 i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
 i_datagram <<= 8;
 i_datagram |= SPI.transfer((datagram) & 0xff);
 digitalWrite(chipCS,HIGH);
 return i_datagram;
//  Serial.print("Received: ");
//  Serial.println(i_datagram,HEX);
//  Serial.print(" from register: ");
//  Serial.println(address,HEX);
}