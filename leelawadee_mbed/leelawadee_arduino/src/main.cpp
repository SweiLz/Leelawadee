#include <Arduino.h>
#include <SoftwareSerial.h>

#define LED_PIN 13



// class MotorAnubis360
// {
// private:
//     int8_t _TXPin,_RXPin;
//     long _baud;
// public:
//     SoftwareSerial _serial;
//     MotorAnubis360(int8_t TxPin,int8_t RxPin);
// };

// MotorAnubis360::MotorAnubis360(int8_t TxPin,int8_t RxPin)
// {
//     _TXPin = TxPin;
//     _RXPin = RxPin;
//     // _baud = Baud;
//     _serial = new SoftwareSerial(_RXPin,_TXPin);
//     // _serial.begin(_baud);
//     // while(!_serial);

// }

// MotorAnubis360 motor(8,9);
SoftwareSerial motor(8,9);
char command[50];

void changeID(int8_t,int8_t);
void setMode(int8_t,int8_t);
void setSpeedRaw(int8_t,int16_t);

void setup() {
    // put your setup code here, to run once:
    pinMode(LED_BUILTIN,OUTPUT);

    Serial.begin(9600);
    Serial.println("Start Serial!!");
    motor.begin(115200);
    while(!motor);
    String txt = "Initialize motor.";
    Serial.println(txt);
    // changeID(1,2);
    // setMode(2,7);

}

int16_t raw_speed=1500;

void loop() {
    // put your main code here, to run repeatedly:
    // digitalWrite(LED_BUILTIN,HIGH);
    // delay(500);
    // digitalWrite(LED_BUILTIN,LOW);
    // delay(500);
    if(Serial.available())
    {
        int16_t speed = Serial.parseInt();
        if(speed > 0)
        setSpeedRaw(2,speed);

    }
}
void setSpeedRaw(int8_t Id,int16_t RawSpeed)
{
    sprintf(command,"#%dP%dT0\r\n",Id,RawSpeed);
    Serial.print(command);
    motor.print(command);
}

void setMode(int8_t Id,int8_t Mode)
{
    sprintf(command,"#%dPMOD%d\r\n",Id,Mode);
    Serial.print(command);
    motor.print(command);
}

void changeID(int8_t Id,int8_t newId)
{
    sprintf(command,"#%dPID%03d\r\n",Id,newId);
    Serial.print(command);
    motor.print(command);
}