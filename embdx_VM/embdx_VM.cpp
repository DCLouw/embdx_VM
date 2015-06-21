
//embdx_VM source code
// Dehan Louw 2015

//Library includes
#include <SPI.h>
#include <avr/io.h>
#include "Adafruit_BLE_UART.h"
#include "Arduino.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <Math.h>

//Framework
void setup();
void loop();

//Global methods

    //System configuration
void configAdc();
void configPins();
void configMegaClk();
void configInts();
void configTimer();

    //Callbacks
void aciCallback(aci_evt_opcode_t event);
void rxCallback(uint8_t *buffer, uint8_t len);

    //I/O
void sendBLE(char[10]);
uint16_t adcRead();

//Global variables

volatile unsigned long tmrOverflowsNow;
volatile unsigned long cadTime;
volatile unsigned int cadRPM;
volatile  unsigned long int runningTotal; //Change type back to unsigned long int after test
volatile int reads;
volatile uint16_t adcAvg;
volatile int c;

unsigned long tmrOverflowsThen0;
unsigned long tmrOverflowsThen1;

uint16_t adcWord;
uint8_t adcByteLow;
uint8_t adcByteHigh;

char cool[] = "goof";
char Datastring[10];
//Macros
#define goBit(reg, target) (reg) |= (1<<(target))
#define noBit(reg, target) (reg) &= ~(1<<(target))

#define ADAFRUITBLE_REQ 0
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 1

//Initialize uart object
Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

void aciCallback(aci_evt_opcode_t event)
{
    
    switch(event)
    {
        case ACI_EVT_DEVICE_STARTED:
            Serial.println(F("Advertising started"));
            break;
        case ACI_EVT_CONNECTED:
            Serial.println(F("Connected!"));
            break;
        case ACI_EVT_DISCONNECTED:
            Serial.println(F("Disconnected or advertising timed out"));
            break;
        default:
            break;
    }
}

void rxCallback(uint8_t *buffer, uint8_t len)
{
    Serial.print(F("Received "));
    Serial.print(len);
    Serial.print(F(" bytes: "));
    for(int i=0; i<len; i++)
        Serial.print((char)buffer[i]);
    
    Serial.print(F(" ["));
    
    for(int i=0; i<len; i++)
    {
        Serial.print(" 0x"); Serial.print((char)buffer[i], HEX);
    }
    Serial.println(F(" ]"));
    
    
    uart.write(buffer, len);
    
}


void setup()
{

Serial.println(F("Starting system setup..."));
    
    //Initialize serial comms
    Serial.begin(9600);
    while(!Serial);
    Serial.println(F("Velometrics embdx running..."));
    
    uart.setRXcallback(rxCallback);
    uart.setACIcallback(aciCallback);
    
    //configPins();
    
    //configMegaClk();
    //configAdc();
    //configInts();
    //configTimer();
    
    uart.begin();
    
Serial.println(F("System setup complete"));
    
    
}


void loop()
{
    //Serial.println(F("Dwelling in main loop"));
    //_delay_ms(500);

    uart.pollACI();
}


void configAdc()
{
Serial.println("adc config started");
    cli();
    
    SPI.begin();
    
    //Resetting the adc
    goBit(PORTD, PORTD6);
    _delay_ms(100);
    noBit(PORTD, PORTD6);
    _delay_ms(100);
    goBit(PORTD, PORTD6);
    
    //Perform initial startup routine
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    noBit(PORTB,PORTB2); //Select the AD7715
    
    SPI.transfer(0x10); //W to Comms reg: Prepare for write to setup register
    SPI.transfer(0x48); //W to Setup reg: 1MhZ clock, self calibration mode, 25KhZ output update rate
    
    goBit(PORTB,PORTB2); //Deselect the AD7715
    SPI.endTransaction(); //Release SPI bus
    sei();
    
Serial.println("adc config completed");
    
}

void configTimer()
{
    
    Serial.println("timer config started");
    
    //Clear registers first
    TCCR2A=0;
    TCCR2B=0;
    
    goBit(TIMSK2, TOIE2); // enable timer/counter2 overflow interrupt
    goBit(TCCR2B, CS22); // running @ /64 pre-scaled clock
    
    
    Serial.println("timer config completed");
    
}


void configInts()
{
    
    Serial.println("interrupts config started");
    
    //INT1 used for DRDY
    goBit(PORTD, PORTD3); //Activates the pull up resistor on pin D3
    goBit(EICRA,ISC11); // Falling edge of INT1 generates an IRQ
    noBit(EICRA,ISC10);
    //goBit(EIMSK, INT1); // Activates INT1
    
    //PCINT0 used for cadence reed switch
    //goBit(PORTB, PORTB0);	//Activates the pull up resistor on pin B0
    //goBit(PCICR, PCIE0);	// Enables PCMSK0 scan
    //goBit(PCMSK0, PCINT0);	//Sets the mask bit for PCINT0
    
    Serial.println("interrupts config completed");
    
}


void configPins()
{
    Serial.println("configPins started");
    
//Datadirection
    //NRF8001
    //goBit(DDRD, DDD1); //Reset
    //goBit(DDRD,  DDD0); //CS
    //noBit(DDRD, DDD2); //RDY
    
    //AD7715
    goBit(DDRD, DDD6); //Reset
    goBit(DDRB,  DDB2); //CS
    //noBit(DDRD, DDD3); //DRDY //IS THIS NECESSARY FOR INT1?
    
    //SPI
    //goBit(DDRB, DDB3); //MOSI
    //noBit(DDRB, DDB4); //MISO
    //goBit(DDRB, DDB5); //SCLK
    
    //Interrupts
    //noBit(DDRB, DDB0); //Cadence
    
    //Clock gen
    goBit(DDRB, DDB1); //1MhZ clock
    
    
//Initial values
    //NRF8001
    //goBit(PORTD, PORTD0); //CS
    
    //AD7715
    goBit(PORTB, PORTB2); //CS
    
    //SPI
   // noBit(PORTB,  PORTB5);  //SCK
   // noBit(PORTB, PORTB3);   //MOSI
    
    Serial.println("configPins completed");

}



//INTERRUPT VECTORS


ISR(INT1_vect) // ISR for new data available
{
    
    runningTotal = adcRead(); //Update the new total thusfar  //ADD += AGAIN AFTER TEST
//    //reads++;
//    
    itoa(runningTotal,Datastring,10);
//
    sendBLE(Datastring);

}

void sendBLE(char* data)
{
   
    Serial.println(data);
    uart.write((uint8_t *)data,10);
    uart.pollACI();
    
}


ISR(TIMER2_OVF_vect) // ISR for timer 2 overflow
{
    
    ++tmrOverflowsNow;
    
    
}

//ISR(PCINT0_vect) //ISR for cadence reed switch
//{
//// Check that the pin change is due to cadence reed being pulled low
//if( ((PINB &(1<<PINB0)) ==0) ) //check if PINB0 is infact low
//{


//if (tmrOverflowsNow > (tmrOverflowsThen1+300)) //Prevents bounce on the cadence reed switch
//{
    
    //Serial.println("interrupt passed");
    
    
    //cadTime = ((tmrOverflowsNow-tmrOverflowsThen1)*1024)+(TCNT1*4); //time in uS
    //cadRPM = round((60000000)/cadTime);

    //adcAvg = round(runningTotal / reads);

    //itoa(adcAvg,Datastring,10);

    //sendBLE(Datastring);

//reads =0;
//runningTotal=0;


//}

//else {
    
//    Serial.println("Bounce detected");
//}//else


//}//If indeed

//}//ISR if



//Timer 1 setup as 1Mhz clock on pin 15 to drive the AD7715
void configMegaClk()
{
    Serial.println("configMegaClk Started");
    
    TCNT1 =0;
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    
    OCR1A = 7;
    
    goBit(TCCR1A, COM1A0); //Toggle OC1A on Compare Match
    
    goBit(TCCR1B, WGM12); //Setting timer1 to CTC mode
    
    goBit(TCCR1B, CS10); //Pre-scaling factor of 1
    
    goBit(DDRB, DDB1); //Set port B1 as output. This pin will generate the clock
    
    Serial.println("configMegaClk Completed");
    
    
}

uint16_t adcRead(){
    
    //cli(); //Disable all interrupts while read is taking place
    
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    noBit(PORTB,PORTB2); //Select the AD7715
    
    //while(digitalRead(3)); //Ensuring that a read is not performed on a high level
    
    Serial.println("Performing read...");
    
    SPI.transfer(0x38); //W to Comms reg: Prepare for read from the data register next
    
    adcByteHigh = SPI.transfer(0); //R from Data reg: First data byte
    adcByteLow = SPI.transfer(0); //R from Data reg: Second data byte
    
    adcWord = ((adcByteHigh<<8) | adcByteLow);
    
    SPI.endTransaction(); //Release SPI bus
    goBit(PORTB,PORTB2); //Deselect the AD7715
    
    //sei(); //Re-anable interrupts
    
    return adcWord;
    
    

}

