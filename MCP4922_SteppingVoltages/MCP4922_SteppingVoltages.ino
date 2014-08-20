
/*
   Created: Chip Audette, Aug 2014
   
   Purpose: Routines to send data via the MCP4922 12-bit, 2-channel DAC
   Approach: The MCP4922 is a SPI device.  Push out 2-byte codes per sample per channel.
   
   Pin3 = (NOT)CS   = Arduino Pin 10 (SS)
   Pin4 = SCK       = Arduino Pin 13 (SCK)
   Pin5 = SDI       = Arduino Pin 11 (MOSI)
   Pin8 = (NOT)LDAC = LOW
   Pin9 = (NOT)SHDN = HIGH
   Pin11 = Pin12 = Vref = HIGH
   
   Pin14 = Vout A
   Pin10 = Vout B
*/

#include <SPI.h>

int slaveSelectPin =10;
int val_chan0 = 0;
int val_chan1 = 0;
#define MAX_VAL (4095)
int step_size = (MAX_VAL / 5);

void setup() {
  // set the slaveSelectPin as an output
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin,HIGH);  //set DAC ready to accept commands

  
  // initialize SPI for MCP4922
  SPI.begin(); 
  SPI.setDataMode(SPI_MODE0); //MCP4922 can be either Mode 0 or Mode 3 (supposedly)
  SPI.setBitOrder(MSBFIRST);
  delay(500);
  
  // start serial
  Serial.begin(115200);
  Serial.println("Test_MCP4922: starting...stepping through voltages...");
  
  //set initial value
  val_chan0 = 0;
  val_chan1 = MAX_VAL;
  Serial.print("Writing "); Serial.print(val_chan0); Serial.print(" "); Serial.println(val_chan1);
  MCP4922_write(slaveSelectPin,val_chan0,val_chan1);  //initial
}

void loop() {
  delay(3000);
  
  //set desired output
  val_chan0 += step_size; //going up
  if (val_chan0 > MAX_VAL) val_chan0 = 0;
  val_chan1 -= step_size; //going down
  if (val_chan1 < 0) val_chan1 = MAX_VAL;
  
  //tell Serial what we're issuing
  Serial.print("Writing "); Serial.print(val_chan0); Serial.print(" "); Serial.println(val_chan1);
  
  //command the new values
  MCP4922_write(slaveSelectPin,val_chan0,val_chan1);  //initial
}

void MCP4922_write(const int &slavePin,const int &value1,const int &value2) {
  int value = 0;
  byte configByte = 0;
  byte data=0;
  int channel=0;
  
  for (channel = 0; channel < 2; channel++) {
    digitalWrite(slavePin,LOW);  //set DAC ready to accept commands
    if (channel == 0) {
      configByte = B01110000; //channel 0, Vref buffered, Gain of 1x, Active Mode
      value = value1;
    } else {
      configByte = B11110000; //channel 1, Vref buffered, Gain of 1x, Active Mode
      value = value2;
    }
    
    //write first byte
    data = highByte(value);
    data = B00001111 & data;  //clear out the 4 command bits
    data = configByte | data;  //set the first four command bits
    SPI.transfer(data);
    
    //write second byte
    data = lowByte(value);
    SPI.transfer(data);
    
    //close the transfer
    digitalWrite(slavePin,HIGH);  //set DAC ready to accept commands
  }
}

