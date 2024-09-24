
//Include Wire I2C library,SPi library, & Adafruit CAN
#include <Wire.h>
#include <SPI.h>
#include "CANController.h"
#include "CANSAME5x.h"
#include "CANSAME5x_port.h"

CANSAME5x CAN;

#define DEBUG 1

//Baudrate is set 
#define Baudrate 9600

//Static addresses for ADC I2C
#define ADC_ADDRESS 0b11010001 // ADC datasheet 5.3.1

#define BytesRequest 2 //Request 2 bytes from the ADC
#define ADCBitSize 12 //predetermined setting for the amount of bits sent
#define MaxBitSize 16 // Max size of the ADC

#define LSBVolt .001  //the voltage of LSB for 12bit
#define PGA 1 //gain on resolution of ADC

//SYNC pinout
#define SYNC 13

#define THERM_MODULE 1
#define THERMISTOR_NUM 16

struct thermistor {
  uint32_t ID = 0;
  int8_t temp = 0;
};

struct CANDATA{
int8_t AVERAGE_TEMP,HIGH_TEMP,HIGH_ID,LOW_TEMP,LOW_ID;
};

// init array of thermistor structs that Store the values of the thermistors ID and there temp
thermistor matrix[THERMISTOR_NUM];

void CalcAVERAGE(int8_t *AVERAGE_TEMP){
int8_t sum;
  for(uint8_t i = 0;i < THERMISTOR_NUM; ++i){
  sum += matrix[i].temp;
  }
  *AVERAGE_TEMP = (sum/THERMISTOR_NUM);
}


//==================================================================================================
void CalcHIGH(int8_t *HIGH_TEMP, int8_t *HIGH_ID){
int8_t Temp,Temp_ID;
  for (uint8_t i = 0;i < THERMISTOR_NUM; ++i){
    if(matrix[i].temp > Temp){

      Temp = matrix[i].temp;
      Temp_ID = i;
   }
  }
  *HIGH_TEMP = Temp;
  *HIGH_ID = Temp_ID;
}
//=================================================================================================

//=================================================================================================
void CalcLOW(int8_t *LOW_TEMP, int8_t *LOW_ID, int8_t *AVERAGE_TEMP){
int8_t Temp = *AVERAGE_TEMP;
int8_t Temp_ID;
  for (uint8_t i = 0;i < THERMISTOR_NUM; ++i ){
    if(matrix[i].temp < Temp){
      Temp = matrix[i].temp;
      Temp_ID = i;
    } 
  }
  *LOW_TEMP = Temp;
  *LOW_ID = Temp_ID;
}
//=================================================================================================

// ================================================================================================

int16_t VtoTforF24(uint16_t x)//
{
    int I =-474.75*pow(x,6)+4616.6*pow(x,5)-17903*pow(x,4)+34634*pow(x,3)-33495*pow(x,2)+13014*x;
    return I;
}

// ================================================================================================

void setup()
{
    int addresswire = 0;
      //Start serial communication at some baud
    Serial.begin(Baudrate);

    // SPI Set up
    pinMode(SYNC,OUTPUT);//sets the MUX sync bit to pin 13
    digitalWrite(SYNC,HIGH);//sets to high to initialize SPI

    // CAN Setup
    pinMode(PIN_CAN_STANDBY,OUTPUT);//CAN Standby 
    digitalWrite(PIN_CAN_STANDBY,false); // turn off STANDBY
    pinMode(PIN_CAN_BOOSTEN,OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN,true); // turn on booster

    uint8_t ADC_CONFIG_R = 144; //set to default ADC config on boot-up, but it's changeable

    //configuration byte for the ADC
    /*
    Consists of bits RDY,C1,C2,O/C,S1,S0,G1,G0

    RDY - in writing initiates a new conversation with 1, but has no effect in in continuous conversion

    C1-C0 - Channel select, so should be left 00

    0/C - Channel Mode; 1 is Continous Conversion mode, 0 is One-shot Conversion mode

    S1-S0 - Sample Rate Selection Bit 
            00 = 240 SPS (12 bits)
            01 = 60 SPS (14 bits)
            10 = 15 SPS (16 bits)

    G1-G0 - PGA Gain Selector Bits
            00 = 1 V/V
            01 = 2 V/V
            10 = 4 V/V
            11 = 8 V/V

    Default config is 10010000, or 144 meaning continuous conversion, at 12 bits, and 1 V/V gain

    */

    Wire.begin(); //intializes I2C
    delay(1);

    //writes to the ADC and sends the config bit

    Wire.beginTransmission(ADC_ADDRESS);
    Wire.write(ADC_CONFIG_R);
    Wire.endTransmission();

    
    SPI.begin(); // Initializes the SPI bus
    delay(1);

    /*
    Code needs to transmit this into a CAN frames
    Frame - 1: Contains the Address Claim at 200 ms and 8 bytes

    B1-B3  -  Identifier for the address claim (default is 0xF30080)

    B4  -  BMS Target Address (default 0xF3)

    B5 -  Thermistor Module(up to 3)

    B6  -  constant (0x40)

    B7  -  constant (0x1E)

    B8 -  constant (0x90)
    */
    uint32_t Identifieraddress = 0xF3;

    if(CAN.begin(5))
    {
        Serial.print("CAN failed to intialize");
    }

    CAN.beginPacket(0x18EEFF80);
    CAN.write(0xF3);
    CAN.write(0x00);
    CAN.write(0x80);
    CAN.write(THERM_MODULE);
    CAN.write(0x40);
    CAN.write(0x1E);
    CAN.write(0x90);
    CAN.endPacket();
    CAN.end();

}

// ================================================================================================

void loop() 
{

    uint8_t AMux = 0; //initializes the address of the Mux at 0

    for(int i = 0; i < THERMISTOR_NUM; ++i)// change this to a for loop that iterates through the matrix and calculates cel
    {
        delay(1);
        
        digitalWrite(SYNC, LOW);// sets to low to send the ack bit to MUX

        //begin SPI request to Mux
        SPI.beginTransaction(SPISettings(30000, MSBFIRST, SPI_MODE1));
      
        /*
        SPI Modes
        0 - Clock Polarity is 0, data cap on Rising
        1- Clock Polarity is 0, data cap on Falling
        2- Clock Polarity is 1, data cap on Falling
        3- Clock Polarity is 1, data cap on Rising
        
        We want spi mode 1 or 2 to cap falling edge, Max Hz is 30,000
        
        */
        SPI.transfer(i);
      
        /*transfers the address byte for MUX

        Consits of bits in MSB EN,CSA,CSB,X,A3,A2,A1,A0
        
        EN - when 1, all switches off

        CSA/CSB - when both 1, retains the previuos Switch Condition

        A0-A3 - Address of Switch begining with 0000 where S1A -> DA, S1B ->DB
                ending with the 1111 where S16A -> DA, S16B -> DB

          For our purpuse, we will start with 0 and increase to a max of 15 to cycle
          all the addresses.

        */
        SPI.endTransaction(); //end SPI
        digitalWrite(SYNC,HIGH); // indicates end of SYNC

        if(!Wire.available()){ //check to see if I2C connects
          Serial.print("I2C failed to conect to ADC");
        }
        //Request 2 bytes from the specified address
        Wire.requestFrom(ADC_ADDRESS,BytesRequest); 
        int16_t remaining = BytesRequest;
        int16_t package;

        while(Wire.available() && --remaining){
        // Get the data in 2 byetes and assign to x and y
          uint8_t x =  Wire.read();
          uint8_t y =  Wire.read();
          package = (y << 8) + x; //combine x and y to get full 16 bit package
        }
        /*
        When in 12-bit mode or 14-bit mode, the ADC will repeat the most sigificant bit after the 12-bit or 14 bit transmission.
        To get rid of those unwanted 1s, a for loop is used to clear those bits.
        */
        uint16_t mask = 1 << (ADCBitSize);// create a mask that is the size of the setted ADC size
        for (uint16_t i = 0; i == MaxBitSize - ADCBitSize ;i++) // intialize i with 0 then goes till 4
        {
          package &= ~(mask);
          mask = mask<<1; //mask gets rid of a leading 1
        }
        int16_t volt = package*(LSBVolt/PGA);// formula to convert the package to volts

        int8_t cel = VtoTforF24(volt);

        matrix[i].temp = cel;
        matrix[i].ID = i;
      #if DEBUG
          //Send the temperature in degrees C an the IDto the serial monitor
          Serial.println(matrix[i].temp);
          Serial.print(matrix[i].ID);
      #endif
    }
    /*
        end the for loop

        return func the average of the matrix

        return func lowest temp 

        return the highest temp and its ID

        */
      int8_t AVERAGE_TEMP,HIGH_TEMP,HIGH_ID,LOW_TEMP,LOW_ID;

      CalcHIGH(&HIGH_TEMP,&HIGH_ID);
      CalcAVERAGE(&AVERAGE_TEMP);
      CalcLOW(&LOW_TEMP,&LOW_ID,&AVERAGE_TEMP);
      

      int16_t LONG_CHECKSUM = AVERAGE_TEMP + HIGH_TEMP + HIGH_ID +
                        LOW_ID + LOW_TEMP + THERM_MODULE +
                        0x39 + 0x08;
      /*Realistically, this value can never go passed around 667
      assuming the all temperature is 120C and all 80 thermistors are enabled
      and we call this module 3
      */
      
      int8_t CHECKSUM;
      if(LONG_CHECKSUM > pow(2,(sizeof(CHECKSUM)*8)-1)-1){
        CHECKSUM = pow(2,(sizeof(CHECKSUM)*8)-1)-1;
      }
      else{
        CHECKSUM = (int8_t) LONG_CHECKSUM;
      }

       /*
        CAN

        Frame - 2: Contains the Thermistor Module Broadcast with 8 bytes at 100 ms

        B1- Thermistor Molde # 

        B2  -  Lowest Thermistor Value (in C)

        B3  -  Highest Thermistor Value (in C)

        B4  -  Average Thermistor Temp (in C)

        B5 -   Number of Thermistors Enable (Max is 80)

        B6 -   Highest Thermistor ID (Zero Based)

        B7  -  Lowest Thermistor ID (Zero Based)

        B8  -  Checksum Byte (add all the bytes + 0x39 + length(8))

        */

        if(CAN.begin(100000)){
        Serial.print("CAN failed to intialize");
        }
        CAN.beginPacket(0x1839F380);
        CAN.write(THERM_MODULE);
        CAN.write(LOW_TEMP);
        CAN.write(HIGH_TEMP);
        CAN.write(AVERAGE_TEMP);
        CAN.write(THERMISTOR_NUM);
        CAN.write(HIGH_ID);
        CAN.write(LOW_ID);
        CAN.write(CHECKSUM);
        CAN.endPacket();
        CAN.end();
    
}


