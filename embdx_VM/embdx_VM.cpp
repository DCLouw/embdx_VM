
//embdx_VM source code
// Dehan Louw 2015

#include <SPI.h>
#include <avr/io.h>

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

void tmr1Config(); //Generates MCLK for adc


//Some macros to aid in bitwise operations
#define goBit(reg, target) (reg) |= (1<<(target))
#define noBit(reg, target) (reg) &= ~(1<<(target))


uint8_t regCommand;
volatile uint8_t regData;
uint16_t adcWord;
uint8_t adcByteHigh;
uint8_t adcByteLow;

//Overheads
#include "Arduino.h"
void setup();
void loop();


void setup()
{
    Serial.begin(9600);
    while(!Serial);
    Serial.println(F("Starting program..."));
   //////////////////////
    
    SPI.begin();
    
    //Configure the IO lines
    pinMode(6, OUTPUT);
    //pinMode(HAL_IO_RADIO_RDY,   INPUT_PULLUP);
    pinMode(10,  OUTPUT);
    
    digitalWrite(6, 1);
    delay(100);
    digitalWrite(6, 0);
//#ifdef __arm__
    delayMicroseconds(10);
//#endif
    digitalWrite(6, 1);
    
    digitalWrite(SCK,  0);
    digitalWrite(MOSI, 0);
    digitalWrite(10,   1);
    digitalWrite(SCK,  0);
    
    ////////////////////
    
    //pinMode(10, OUTPUT); //Ensuring SS is output, to keep micro in master mode
    pinMode(7, INPUT);
    
    tmr1Config(); //Generates the clock for MCLK of the adc
    
    
    //_delay_ms(200);
    Serial.println(F("Setup is done"));
    //_delay_ms(100);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    //_delay_ms(200);
    digitalWrite(10,   0);
    
    
    SPI.transfer(0x10); //Prepare for write to setup register
    //_delay_ms(100);
    SPI.transfer(0x48); //1MhZ clock, self calibration mode, 25KhZ output update rate
    

}


void loop()
{
    
    
    //Serial.println("waiting for while");
    while (digitalRead(7)); //Wait for DRDY to drop low
    
        //Serial.println("DRDY has been dropped");
    SPI.transfer(0x38); //Prepare for read from the data register next
    
    adcByteHigh = SPI.transfer(0); //Read first byte from Data reg.
    adcByteLow = SPI.transfer(0); //Read second byte from Data reg.
    
//    Serial.println("The first byte was:");
//    Serial.println(adcByteOne);
//    Serial.println("The seconds byte was:");
//    Serial.println(adcByteTwo);
//    Serial.println ("--");
    
  
    adcWord = ((adcByteHigh<<8) | adcByteLow);
    
    Serial.println(adcWord);
    
    //SPI.endTransaction();
    
    //_delay_ms(2000);
    
}




//Timer 1 setup as 1Mhz clock on pin 15 to drive the AD7715
void tmr1Config()
{
    
    TCNT1 =0;
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    
    OCR1A = 7;
    
    goBit(TCCR1A, COM1A0); //Toggle OC1A on Compare Match
    
    goBit(TCCR1B, WGM12); //Setting timer1 to CTC mode
    
    goBit(TCCR1B, CS10); //Pre-scaling factor of 1
    
    goBit(DDRB, DDB1); //Set port B1 as output. This pin will generate the clock
    
    
}

