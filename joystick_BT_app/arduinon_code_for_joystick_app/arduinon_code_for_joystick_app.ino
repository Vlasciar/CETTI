//Author: TableTop Robotics

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


#define SERVOMIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;
int dataIn[5] {0,0,0,0}; //array to store all the information. 255,button,X,Y.
int in_byte = 0;
int array_index = 0;
int pulselength;

void setup() {
Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
 // Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

int cooldown=0;
void loop() {
  // put your main code here, to run repeatedly:
if (SerialBT.available() > 0) {  //recieve byte from phone
  in_byte = SerialBT.read(); //store in byte into a variable
  if (in_byte == (255)) { // if the variable is 0 stet the array inxed to 0. this will make sure that every number goes into the correct index
    array_index = 0;
  }
  dataIn[array_index] = in_byte;  //store number into array
  array_index = array_index +1;
}
//print the array
Serial.print (dataIn[0]);
Serial.print (", button:");
Serial.print (dataIn[1]);
Serial.print (", X:");
Serial.print (dataIn[2]);
Serial.print (", Y:");
Serial.print (dataIn[3]);
Serial.print (",");
Serial.println (in_byte);
  pulselength = map(dataIn[2], 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0, 0, pulselength);
  
  pulselength = map(dataIn[3], 0, 180, SERVOMIN, SERVOMAX);
 pwm.setPWM(1, 0, pulselength);


}
