
//embdx_VM source code
// Dehan Louw 2015

#include <SPI.h>
#include <avr/io.h>
#include "Adafruit_BLE_UART.h"

#include <util/delay.h>
#include <avr/interrupt.h>


//SPI driver methods and variables
void spiConfig(int);

#define NRF8001 1
#define AD7715 2


//Timing
void timerConfig();
void tmr1Config();

volatile unsigned long tmrOverflowsNow;
volatile unsigned long cadTime;
volatile unsigned int cadRPM;
unsigned long tmrOverflowsThen0;
unsigned long tmrOverflowsThen1;

//Interrupts
void intConfig();

//ADC
void adcConfig();


//Some macros to aid in bitwise operations
#define goBit(reg, target) (reg) |= (1<<(target))
#define noBit(reg, target) (reg) &= ~(1<<(target))

//AD7715 methods and variables
//uint16_t adcRead();

uint8_t regCommand;
volatile uint16_t regData;
uint16_t adcval;
uint8_t adctemp;
volatile int c;

//Test data
char cool[] = "goof";
char twocool[5] ;
char threecool[5] ;
char fourcool[5] ;
char fivecool[5] ;
char sixcool[10] ;
uint8_t lengthy =2;

//Overheads
#include "Arduino.h"
void aciCallback(aci_evt_opcode_t event);
void rxCallback(uint8_t *buffer, uint8_t len);
void setup();
void loop();
void gogowrite();

#define ADAFRUITBLE_REQ 4 //PD4
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 7

//other important pins
//ADC CS on PD5
//ADC RESET on PD6


//#line 25
//Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

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
    
    
    //uart.write(buffer, len);
    
}


void setup()
{
    Serial.begin(9600);
    while(!Serial);
    Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Callback Echo demo"));
    
    
    tmrOverflowsNow =0;
    tmrOverflowsThen1=0;
    
    
    //Set CS pins for both chips as outputs
    //goBit(DDRD,DDD5); //CS for ADC
    //goBit(PORTD, PORTD5); //Making sure ADC chip is unselected right from the start
    
    //goBit(DDRD, DDD6); // Control for adc RESET pin
    //goBit(PORTD, PORTD6); // Reset pulled high by default
    //little init verificatioln sequency
   
    pinMode(SS, OUTPUT);
    
    SPI.begin();
//    _delay_ms(500);
//    noBit(PORTD, PORTD6);
//    _delay_ms(500);
//    goBit(PORTD, PORTD6);
//    _delay_ms(500);
//    noBit(PORTD, PORTD6);
//    _delay_ms(500);
//    goBit(PORTD, PORTD6);
//    _delay_ms(500);
    
    
    //timerConfig();
    //_delay_ms(35);
    tmr1Config();
    
    intConfig();
    
    //adcConfig();
    
    //itoa (2,twocool,10);
    
    //sprintf(fourcool,"%d",4);
    //sprintf(fivecool,"%d",0b00000101);
    //sprintf(sixcool,"%d",adctemp);
    
    
    //uart.setRXcallback(rxCallback);
    //uart.setACIcallback(aciCallback);
    
    //uart.begin();
}


void loop()
{
    //uart.pollACI();
    
    
    
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    
    adctemp = SPI.transfer(0b00001000);
    _delay_ms(100);

    
    Serial.println(adctemp);
    Serial.println ("--");
    
    SPI.endTransaction();
    
    _delay_ms(100);
    
////
    
//    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//    
//    adctemp = SPI.transfer(0b00000100);
//    
//    Serial.println(adctemp);
//    Serial.println ("second read result");
//    
//    SPI.endTransaction();
//    
//    _delay_ms(2000);
    
    ////
//    
//    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
//    
//    adctemp = SPI.transfer(0b00001100);
//    
//    Serial.println(adctemp);
//    Serial.println ("third read result");
//    
//    SPI.endTransaction();
//    
//    _delay_ms(2000);
    
    ////
    
//    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
//    
//    adctemp = SPI.transfer(0b00001000);
//    
//    Serial.println(adctemp);
//    Serial.println ("Fourth read result");
//    
//    SPI.endTransaction();
//    
//    _delay_ms(700);
    
    ////
    
    
    
}


void adcConfig()
{
    //cli();
    
    Serial.println("adc config started");
    
    //Configure custom SPI settings
    //spiConfig(AD7715);
    
    //goBit(DDRD, DDD4);
    //goBit(PORTD, PORTD4); //unselecting the nrf8001
    
////////////////////// FIRST ATTEMPT
    
//    SPI.beginTransaction(SPISettings(62500, MSBFIRST, SPI_MODE2));
//    _delay_ms(1000);
    //noBit(PORTD, PORTD5); //Select the ADC
//
    SPI.begin();
    
//    noBit(PORTD, PORTD6); // ADC is reset
//    _delay_ms(500);
//    goBit(PORTD, PORTD6); // Reset pulled high
//    _delay_ms(1000);
//    
//    //Communications register write
    regCommand = 0b00001000; //just reading from the comms register for the second operation
//    adctemp = SPI.transfer(regCommand);
//   //adctemp = SPI.transfer(0);
//    _delay_ms(1000);
//    itoa (adctemp, twocool,10);
//    Serial.println(regCommand);
//    Serial.println ("was sent to ad7715");
//    goBit(PORTD, PORTD5);
//    SPI.endTransaction();
//    _delay_ms(500);
    
/////////////////////////////
    
//    goBit(PORTD, PORTD4); //unselecting the nrf8001
//    noBit(PORTD, PORTD5); //Select the ADC
//    
//    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE2));
//    _delay_ms(1000);
//    noBit(PORTD, PORTD5); //Select the ADC
//    
//    //Communications register write
//    _delay_ms(1000);
//    adctemp = SPI.transfer(8);
//    _delay_ms(500);
//    itoa (adctemp, threecool,10);
//
//    goBit(PORTD, PORTD5);
//    SPI.endTransaction();
//    _delay_ms(500);
    
/////////////////////////////
    

    /////////////////////////////
    
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    noBit(PORTD, PORTD5); //Select the ADC
    //_delay_ms(200);
    adctemp = SPI.transfer(233);
//_delay_ms(500);
    //_delay_ms(200);
    
    //Communications register write
    //_delay_ms(1000);
    //adctemp = SPI.transfer(regCommand);
    //_delay_ms(200);
    //itoa (adctemp, fourcool,10);
    goBit(PORTD, PORTD5);
    SPI.endTransaction();
    _delay_ms(200);
    
    //spi.end();
    
    
    /////////////////////////////

    
    /////////////////////////////
    
//    SPI.beginTransaction(SPISettings(62500, MSBFIRST, SPI_MODE2));
//    _delay_ms(1000);
//    noBit(PORTD, PORTD5); //Select the ADC
//    
//    //Communications register write
//    _delay_ms(1000);
//    adctemp = SPI.transfer(8);
//    _delay_ms(500);
//
//    itoa (adctemp, fivecool,10);
//
//    goBit(PORTD, PORTD5);
//    SPI.endTransaction();
//    _delay_ms(500);
    
    /////////////////////////////

    ///////The actual metod to be inserted now to replace the dummy read of the comms register above
    //regCommand = 0b00010000; // Prepare for a write to the setup register next. set gain as 1 initially
    //Setup register write
    //regCommand = 0b01001111; //Self calibration mode. Master clock frequency @1MHz.6.55Hz -3dB cutoff frequency for LP filter. //Unipolar operation. Buffer on?.fsncoff.
    //SPI.transfer(regCommand);
    
    ///////
    
    
    Serial.println(adctemp);
    Serial.println ("was read from ad7715");
    
    //As you were...
    
    Serial.println("adc config completed");
    
    //sei();
}

void timerConfig()
{
    //spiConfig(NRF8001);
    
    Serial.println("timer config started");
    
    TCCR2A=0;
    TCCR2B=0;
    
    goBit(TIMSK2, TOIE2); // enable timer/counter2 overflow interrupt
    goBit(TCCR2B, CS22); // running @ /64 pre-scaled clock
    
    
    Serial.println("timer configg completed");
    
}


void intConfig(void)
{
    
    Serial.println("interrupts config started");
    
    //Interrupt for cadence reed switch
    noBit(DDRD, DDD3);//Configures pin PD3 as an input  equal to DDRD &= ~(1 << DDD3);
    
    goBit(PORTD, PORTD3); //Activates the pull up resistor on pin D3
    
    goBit(EICRA,ISC11); // Falling edge of INT1 generates an IRQ
    
    goBit(EIMSK, INT1); // Activates INT1
    
    //Interrupt for DRDY of the AD7715
    //noBit(DDRB, DDB0);		//Configures pin PB0 as an input
    //goBit(PORTB, PORTB0);	//Activates the pull up resistor on pin B0
    //goBit(PCICR, PCIE0);	// Enables PCMSK0 scan
    //goBit(PCMSK0, PCINT0);	//Sets the mask bit for PCINT0
    
    Serial.println("interrupts config completed");
    
}



//void spiConfig(int dev)
//{
//    
//    switch(dev)
//    {
//            
//        case 1: //nrf8001 SPI settings
//        {
//            //Deselect both slaves
//            goBit(PORTD, PORTD5);
//            //goBit(PORTB, PORTB2);
//            
//            //Setup SPI
//            //SPI.setBitOrder(LSBFIRST);
//            //SPI.setDataMode(SPI_MODE0);
//            //SPI.setClockDivider(SPI_CLOCK_DIV8);
//            Serial.println("spi has been setup for nrf8001");
//            
//            SPI.beginTransaction(SPISettings(1000000, LSBFIRST, SPI_MODE0));
//            
//            //Select NRF8001
//            //noBit(PORTB, PORTB2);
//            break;
//        }
//            
//        case 2: //AD7715 SPI settings
//        {
//            //Deselect both slaves
//            goBit(PORTD, PORTD5);
//            goBit(PORTB, PORTB2);
//            
//            //Setup SPI
//            //SPI.setBitOrder(MSBFIRST);
//            //SPI.setDataMode(SPI_MODE0);
//            //SPI.setClockDivider(SPI_CLOCK_DIV16);
//            Serial.println("spi has been setup for ad7715");
//            
//            SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//            
//            //Select the AD7715
//            noBit(PORTD, PORTD5);
//            break;
//        }
//            
//    }
//    
//    
//}





//uint16_t adcread()
//{
//    
//    Serial.println("adc read started");
//    
//    uint8_t tempbyte;
//    uint16_t tempword;
//    
//    spiConfig(AD7715);
//    
//    //prepare for read operation
//    regCommand = 0b00111000; // prepare for a read of the data register. gain setting of 1.
//    tempbyte = SPI.transfer(regCommand); //Transfers the command and reads the first byte.
//    tempword = tempbyte<<8;
//    tempbyte = SPI.transfer(0); //read the second byte from data register
//    tempword |= tempbyte; // Should now contain full 16 bit data register
//    
//    spiConfig(NRF8001);
//    
//    return tempword;
//    
//}



//INTERRUPT VECTORS


ISR(INT1_vect) // ISR for cadence sensing reed switch
{
    
    
    if (tmrOverflowsNow > (tmrOverflowsThen1+200))
    {
        
        Serial.println("interrupt passed");
        
        
        //cadTime = ((tmrOverflowsNow-tmrOverflowsThen1)*1024)+(TCNT1*4); //time in uS
        //cadRPM = round((60000000)/cadTime);
        
        //adcval = adcread();
        
        //Serial.println("Current power figure is: ");
        //Serial.println(PowNow);
        
        gogowrite();
        
       // uart.write((uint8_t *)sixcool,10);
        
        tmrOverflowsThen1 = tmrOverflowsNow; // set new overflows count to compare next time
        
        Serial.println("interrupt completed");
        
    
    }
    
    else {
        
        Serial.println("F A L S E  A L A R M");
    }
    
    
}

void gogowrite()
{
    cli();
    
    //uart.write((uint8_t *)cool,5);
    //uart.pollACI();
    //uart.write((uint8_t *)threecool,5);
   // uart.write((uint8_t *)twocool,5);
    //uart.pollACI();
     //uart.write((uint8_t *)fourcool,5);
    
    //uart.write((uint8_t *)fivecool,5);
    
    sei();
}


ISR(TIMER2_OVF_vect) // ISR for timer 2 overflow
{
    
    ++tmrOverflowsNow;
    
    
}

//ISR(PCINT0_vect) //ISR for polling DRDY pin
//{
//// Check that the pin change is due to DRDY being pulled low
//if( !((PINB &(1<<PINB0)) ==1) ) //check if PINB0 is infact low
//{

//PowNow = adcRead();

//Just add up to here for now



//This part was written so long for average power val calcs. LEAVE FOR NOW. JUST USE MOMENTARY VAL
//regdata += adcRead();
//c++;
//PowNow = regData / c;

//}

//}



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

