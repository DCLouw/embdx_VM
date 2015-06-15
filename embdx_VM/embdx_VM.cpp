

#include <SPI.h>
#include <avr/io.h>
#include "Adafruit_BLE_UART.h"

#include <util/delay.h>
#include <avr/interrupt.h>


//SPI driver methods and variables
//void spiConfig(int);

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
//void adcConfig();
uint16_t adcRead();

uint8_t regCommand;
volatile uint16_t regData;
uint16_t adcval;
volatile int c;

//Test data
char cool[] = "goof";
uint8_t lengthy =2;

//Overheads
#include "Arduino.h"
void aciCallback(aci_evt_opcode_t event);
void rxCallback(uint8_t *buffer, uint8_t len);
void setup(void);
void loop();

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2
#define ADAFRUITBLE_RST 7


//#line 25
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

void setup(void)
{
    Serial.begin(9600);
    while(!Serial);
    Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Callback Echo demo"));
    
    uart.setRXcallback(rxCallback);
    uart.setACIcallback(aciCallback);
    
    //cool[0] = 6;
    //cool[1] = 5;
    //cool[2] = 2;
    
    tmrOverflowsNow =0;
    tmrOverflowsThen1=0;
    
    uart.begin();
    
    
    intConfig();
    timerConfig();
    tmr1Config();
    adcConfig();
}


void loop()
{
    uart.pollACI();
}


void timerConfig()
{
    
    
    Serial.println("timer config started");
    
    TCCR2A=0;
    TCCR2B=0;
    
    goBit(TIMSK2, TOIE2); // enable timer/counter2 overflow interrupt
    goBit(TCCR2B, CS22); // running @ /64 pre-scaled clock
    
    
    Serial.println("timer config completed");
    
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



void spiConfig(int dev)
{
    
    switch(dev)
    {
            
        case NRF8001: //nrf8001 SPI settings
        {
            //select the nrf8001
            goBit(PORTD, PORTD5);
            noBit(PORTB, PORTB2);
            
            //Setup nrf8001 spi settings
            SPI.setBitOrder(LSBFIRST);
            SPI.setDataMode(SPI_MODE0);
            SPI.setClockDivider(SPI_CLOCK_DIV8);
            Serial.println("spi has been setup for nrf8001");
        }
            
        case AD7715: //AD7715 SPI settings
        {
            //select the AD7715
            noBit(PORTD, PORTD5);
            goBit(PORTB, PORTB2);
            
            //Setup AD7715 spi settings
            SPI.setBitOrder(MSBFIRST);
            SPI.setDataMode(SPI_MODE0);
            SPI.setClockDivider(SPI_CLOCK_DIV8);
            Serial.println("spi has been setup for ad7715");
        }
            
    }
    
    
}


void adcConfig()
{
    Serial.println("adc config started");
    
    
    //Set CS pins for both chips as outputs
    goBit(DDRD,DDD5);
    goBit(DDRB,DDB2);
    
    //Configure custom SPI settings
    spiConfig(AD7715);
    
    //Communications register write
    regCommand = 0b00001000; //just reading from the comms register for the second operation
    adcval = SPI.transfer(regCommand);
    Serial.println(regCommand);
    Serial.println ("was sent to ad7715");
    
///////The actual metod to be inserted now to replace the dummy read of the comms register above
    //regCommand = 0b00010000; // Prepare for a write to the setup register next. set gain as 1 initially
    //Setup register write
    //regCommand = 0b01001111; //Self calibration mode. Master clock frequency @1MHz.6.55Hz -3dB cutoff frequency for LP filter. //Unipolar operation. Buffer on?.fsncoff.
    //SPI.transfer(regCommand);
    
///////
    
    //adcval = SPI.transfer(6);
    Serial.println(adcval);
    Serial.println ("was read from ad7715");
    
    
    //As you were...
    spiConfig(NRF8001);
    Serial.println("adc config completed");
    
}


uint16_t adcread()
{
    
    Serial.println("adc read started");
    
    uint8_t tempbyte;
    uint16_t tempword;
    
    spiConfig(AD7715);
    
    //prepare for read operation
    regCommand = 0b00111000; // prepare for a read of the data register. gain setting of 1.
    tempbyte = SPI.transfer(regCommand); //Transfers the command and reads the first byte.
    tempword = tempbyte<<8;
    tempbyte = SPI.transfer(0); //read the second byte from data register
    tempword |= tempbyte; // Should now contain full 16 bit data register
    
    spiConfig(NRF8001);
    
    return tempword;
    
}



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
        
        uart.write((uint8_t *)cool,5);
        
        tmrOverflowsThen1 = tmrOverflowsNow; // set new overflows count to compare next time
        
        Serial.println("interrupt completed");
    }
    
    else {
        
        Serial.println("F A L S E  A L A R M");
    }
    
    
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



//Timer 1 setup as 1Mhz clock to drive the AD7715
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
    
    
};

// AT last test:

//So succesfull reads were done from the comms register as proof of concept
//metohds have now been added to actual setup and read from data register.
//BY uncommneting necessary lines: DRDY will be monitored and new bytes read each time available
// Once the button is pressed, the current value (stored in volatile pownow) will be displayed
