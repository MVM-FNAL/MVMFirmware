

#define SCL_PIN 2
#define SCL_PORT PORTD
#define SDA_PIN 3
#define SDA_PORT PORTD
#define I2C_TIMEOUT 100
#define I2C_FASTMODE 0

#include <SoftI2CMaster.h>
#include <SoftWire.h>

#include <Wire.h>

#define ALARM_PIN_TOO_LOW 0
#define ALARM_PIN_TOO_HIGH 1
#define ALARM_TEMP_LOW 2
#define ALARM_TEMP_HIGH 3
#define ALARM_ACTIVITY 4
#define ALARM_POWER 5
#define ALARM_CHARGE_CRITICAL 6




SoftWire swire = SoftWire();


#include "ads1115_soft.h"

#define VALVE_THREE_WAY 8
#define VALVE_ANALOG A6
#define BRETHE 6
#define POWER_STATUS 9
#define PROBE_12VSYS A1
#define PROBE_5V_RASP A2
#define PROBE_BCHARGE A3
#define NTC_PIN A7
#define ALARM_RELE 4  
#define ALARM_RED 7
#define ALARM_YELLOW 10
#define ALARM_BUZZER A0


bool EnableWdogController=false;
bool Run=false;
uint8_t BreetheRate = 15;
uint8_t dP = 5;
uint8_t Pin_max = 7;
uint8_t Pin_min = 3;

  int pv2; 
  int breethe; 
  int pwall; 
  int p12v; 
  int p5rasp;
  int pv1; 
  int pbattery; 
  float ntc;
  
  float InPress;
  float Ref25;
  float p33v;
  
float ReadNTC();

uint32_t wdogtimer=0;

bool wdog_alarm=false;
bool power_alarm=false;
bool in_pressure_alarm=false;
bool circuit_alarm=false;
bool activity_alarm=false;

uint16_t ALARM_FLAG = 0;

uint32_t GenerateFlag(int alarm_code)
{
    return (1 << alarm_code);
}


void setup() {
  Serial.begin(115200);  

  digitalWrite(ALARM_RELE, 1);
  digitalWrite(ALARM_RED, 0);
  digitalWrite(ALARM_YELLOW, 0);
  digitalWrite(ALARM_BUZZER, 0);
  
  pinMode(ALARM_RELE, OUTPUT);
  pinMode(ALARM_RED, OUTPUT);
  pinMode(ALARM_YELLOW, OUTPUT);
  pinMode(ALARM_BUZZER, OUTPUT);

  pinMode(VALVE_THREE_WAY, INPUT);
  pinMode(BRETHE, INPUT);
  pinMode(POWER_STATUS, INPUT);

  
  
  // put your setup code here, to run once:
  Wire.begin(0x22);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);  

  
}

  
void loop() {

  // put your main code here, to run repeatedly:
  pv2 = digitalRead(VALVE_THREE_WAY);
  breethe = digitalRead(BRETHE);
  pwall = digitalRead(POWER_STATUS);
  p12v = ((float)analogRead(PROBE_12VSYS)/1024.0)*3.3*5.0*0.93;
  p5rasp = ((float)analogRead(PROBE_5V_RASP)/1024.0)*3.3*2.0;
  pv1 = analogRead(VALVE_ANALOG);
  pbattery = analogRead(PROBE_BCHARGE)/10.0;
  ntc = ReadNTC();

  InPress = (((float)readADC_SingleEnded(0x48,0))* 0.0009175277) - 0.2715030471;
  Ref25 =   readADC_SingleEnded(0x48,2);
  p33v =    ((float)readADC_SingleEnded(0x48,3))/Ref25 *2.5 * 2;

  Serial.println(
    "Run: " + String(Run) + 
    " pv1: " + String(pv1) + 
  " pv2: " + String(pv2) + 
  " breethe: " + String(breethe) + 
  " pwall: " + String ( pwall ) + 
  " p12v: " + String (p12v) + 
  " p5rasp: " + String(p5rasp) +
  " pbattery: " + String(pbattery)+
  " ntc: " + String(ntc) +
  " InPress: " + String(InPress) +
  " Ref25: " + String(Ref25) +
  " p33v: " + String(p33v) +
  " ALARM_FLAG: " + String(ALARM_FLAG)
  
  );

  //Allarmi solo in RUN
  if (Run)
  {
    if (InPress < Pin_min)
      ALARM_FLAG = ALARM_FLAG | GenerateFlag(ALARM_PIN_TOO_LOW);
    else
      ALARM_FLAG = ALARM_FLAG & ~GenerateFlag(ALARM_PIN_TOO_LOW);

    if (pbattery<5.0)
      ALARM_FLAG = ALARM_FLAG | GenerateFlag(ALARM_CHARGE_CRITICAL);  
    else
      ALARM_FLAG = ALARM_FLAG & ~GenerateFlag(ALARM_CHARGE_CRITICAL);                          
        
  }
  else
  {

      ALARM_FLAG = ALARM_FLAG & ~GenerateFlag(ALARM_PIN_TOO_LOW);
      ALARM_FLAG = ALARM_FLAG & ~GenerateFlag(ALARM_CHARGE_CRITICAL);                          
  }   
      
    if (InPress > Pin_max)
      ALARM_FLAG = ALARM_FLAG | GenerateFlag(ALARM_PIN_TOO_HIGH);  
    else
      ALARM_FLAG = ALARM_FLAG & ~GenerateFlag(ALARM_PIN_TOO_HIGH);   

    if (ntc < 5)
      ALARM_FLAG = ALARM_FLAG | GenerateFlag(ALARM_TEMP_LOW);  
    else
      ALARM_FLAG = ALARM_FLAG & ~GenerateFlag(ALARM_TEMP_LOW);  

    if (ntc > 80)
      ALARM_FLAG = ALARM_FLAG | GenerateFlag(ALARM_TEMP_HIGH);  
    else
      ALARM_FLAG = ALARM_FLAG & ~GenerateFlag(ALARM_TEMP_HIGH); 
      
    if ((p12v > 14.5) || (p12v < 10) || (p33v < 3.15) || (p33v > 3.4) || (Ref25 < 19000.00) || (Ref25 > 21000.00))  //aggiungere check raspberry
      ALARM_FLAG = ALARM_FLAG | GenerateFlag(ALARM_POWER);  
    else
      ALARM_FLAG = ALARM_FLAG & ~GenerateFlag(ALARM_POWER);


if (ALARM_FLAG!=0)
{
  digitalWrite(ALARM_RELE, 0);
  digitalWrite(ALARM_RED, 1);
  digitalWrite(ALARM_BUZZER, 1);
}
else
{
  digitalWrite(ALARM_RELE, 1);
  digitalWrite(ALARM_RED, 0);
  digitalWrite(ALARM_BUZZER, 0); 
 
  
}
  //Allarmi sempre attivi


  delay(25);
  

}



//Gestione I2C
unsigned int iic_int_addr;
unsigned int iic_cast;
//Ricezione di byte
void receiveEvent(int howMany) {
    int x;
    int status=0;
    
    unsigned char databuffer[4];
    int dba=0;

    while (0 < Wire.available()) {
        uint8_t c = Wire.read();


        switch(status)
        {
            case 0:  iic_int_addr=c;  break;
            case 1:  databuffer[dba++] = c; break;
            case 2:  databuffer[dba++] = c;

              if (iic_int_addr==0x00)
              {
                wdogtimer=millis();
              }
              if (iic_int_addr==0x01)
              {
                EnableWdogController = databuffer[0];
              }

              if (iic_int_addr==0x02)
              {
                Run = databuffer[0];
              }

              if (iic_int_addr==0x03)
              {
                BreetheRate = databuffer[0];
              }

              if (iic_int_addr==0x04)
              {
                dP = databuffer[0];
              }

              if (iic_int_addr==0x05)
              {
                Pin_max = databuffer[0];
              }

              if (iic_int_addr==0x06)
              {
                Pin_min = databuffer[0];
              }
                    
            break;
            

        }

        status++;
    }

}




  
void requestEvent() {
    
    char c;
    uint16_t value;
    uint8_t databuffer[4];

    if (iic_int_addr == 0x50)
    {
        value = (uint16_t) (InPress>0?InPress:0) * 1000;
    }

    if (iic_int_addr == 0x51)
    {
        value = (uint16_t) (pbattery>0?pbattery:0) ;
    }   

    if (iic_int_addr == 0x52)
    {
        value = pwall ;
    }    

    if (iic_int_addr == 0x53)
    {
        value = p12v*1000.0 ;
    }  

    if (iic_int_addr == 0x54)
    {
        value = p5rasp*1000.0 ;
    }  
    
    if (iic_int_addr == 0x55)
    {
        value = p33v*1000.0 ;
    }  

    if (iic_int_addr == 0x56)
    {
        value = ntc*10.0 ;
    }    

    if (iic_int_addr == 0x57)
    {
        value = ALARM_FLAG ;
    }        


    if (iic_int_addr == 0xFC)
    {
        value = 0x2020 ;
    }    

    if (iic_int_addr == 0xFD)
    {
        value = 0x1804 ;
    }              
    
    databuffer[0] = value&0xFF;
    databuffer[1] = (value>>8)&0xFF;

       
    Wire.write(databuffer[0]);
    Wire.write(databuffer[1]);
}


float ReadNTC(){

  float Rfissa = 10000;  // [ohm]     Resistencia fija del divisor
  float R25 = 10000;    // [ohm]     Valor de NTC a 25�C
  float Beta = 3490.0; // [K]      Par�metro Beta de NTC
  float Beta2 = 3434.0; // [K]      Par�metro Beta de NTC
  float T0 = 298.15;   // [K]       Temperatura de referencia en Kelvin
  float Vtmp = 0.0;    // [V]       Variable para almacenar Vout
  float Rntc = 0.0;    // [ohm]     Variable para NTC en ohmnios
  float TempK = 0.0;   // [K]       Temperatura salida en Kelvin
  float TempC = 0.0;   // [�C]      Temperatura salida en Celsius

  Vtmp=(5.0/1024)*(analogRead(NTC_PIN));
  Rntc=(Vtmp*Rfissa)/(5.0-Vtmp);
  TempK=Beta/(log(Rntc/R25)+(Beta/T0));
  TempC=TempK-273,15;
  return TempC;
}
