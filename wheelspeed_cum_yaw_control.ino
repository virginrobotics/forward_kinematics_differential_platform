//libraries
#include <Wire.h>;
#include <PID_v1.h>
#include "TimerOne.h"
#include <MPU6050.h>

//gyroscope
MPU6050 mpu;
float error;
float timeStep = 0.01;
float yaw, pitch, roll;

//motor control
int right_speed,left_speed;

//encoders
const unsigned long stableTime = 200; //µs
const byte sensorPin = 2;
const byte sensorPin2 = 3;
volatile unsigned long previousTime,previousTime2, count, count2,totalcount2,totalcount;
unsigned long timer,elapsed;
volatile float rotation;
volatile float rotation2;


//motorpid
double Setpoint, Input, Output=0, Setpoint2, Input2, Output2=0;
PID myPID(&Input, &Output, &Setpoint,0,0,0, DIRECT);// 9 13
PID myPID2(&Input2, &Output2, &Setpoint2,6.0,12.75,0.05, DIRECT); // 10  7

//yaw angles
float getYaw() {
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  /*pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
 
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);  
  Serial.print(" Yaw = ");*/
   yaw = (yaw + norm.ZAxis * timeStep); 
  //Serial.println(yaw);
  delay((timeStep*1000) - (millis() - timer));
  
}



void timerIsr() {
  Timer1.detachInterrupt(); 
  digitalWrite(13,HIGH);
  rotation = (count/42.00)*6.66;
  rotation2 = (count2/42.00)*6.66;
  //myPID.Compute();
  
  count2 = 0;
  count = 0;
  Timer1.attachInterrupt( timerIsr );  
}

void sensor_isr()
{
  unsigned long currentTime = micros();
  if (currentTime - previousTime33 >= stableTime) count++;
  previousTime332 = currentTime;
}

void sensor_isr2()
{
  unsigned long currentTime = micros();
  if (currentTime - previousTime33 >= stableTime) count2++;
  previousTime33 = currentTime;
  //count2++;
}

int ena = 6;
int a = 7;
int b = 8;
int enb = 5;
int c = 4;
int d = 12;

void publish_rpm() {
  
  Serial.print(Input2);
  Serial.print(",");
  Serial.print(error);
  Serial.print(",");
  Serial.println(2.00);
  //delay(250);
}

void engage(double arg) {
    right_speed = 0 + arg;
    left_speed = 0 - arg; 
    int left_pwm = 30+abs(arg);
    int right_pwm = 30+abs(arg);
    analogWrite(ena,left_pwm);
    if (left_speed >= 0) {
      digitalWrite(a,1);
      digitalWrite(b,0);
    }

    else {
      digitalWrite(a,0);
      digitalWrite(b,1);
    }
    
    
    analogWrite(enb,right_pwm);
     if (right_speed >= 0) {
      digitalWrite(c,1);
      digitalWrite(d,0);
    }

    else {
      digitalWrite(c,0);
      digitalWrite(d,1);
    }
    
}


/*void tune() {
  if (Serial.available() > 0) {
    String val;
    val = Serial.read();
    Serial.println(val);
    Serial.write(val);
  }
}*/

void pid_loop() {
   while(true) { 
    getYaw();
    Input = rotation2;
    Input2 = yaw*16.39;
    myPID.Compute();
    myPID2.Compute();
    engage(Output2);
    publish_rpm();
    error = abs(Output2)+ abs(Setpoint2-Input2);
    if (error <= 1.5) break;
    delay(150);
   }
}
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Setpoint = 0.00;
Setpoint2 = -45.00;

myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(0,150);
myPID2.SetMode(AUTOMATIC);
myPID2.SetOutputLimits(-80,80);

 while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  { 
    Serial.println("Where's the fuckign sensor asshole. XHECK THE DAMN WIRING!");
    delay(500);
  }
mpu.calibrateGyro();
mpu.setThreshold(3);

delay(2000);
attachInterrupt(0, sensor_isr, RISING);
attachInterrupt(1, sensor_isr2, RISING);
Timer1.initialize(150000);
Timer1.attachInterrupt(timerIsr);


pinMode(ena, OUTPUT);
pinMode(a, OUTPUT);
pinMode(b, OUTPUT);
pinMode(enb, OUTPUT);
pinMode(c, OUTPUT);
pinMode(d, OUTPUT);
analogWrite(ena, 55);
digitalWrite(a, 1);
digitalWrite(b, 0);
analogWrite(enb, 0);
digitalWrite(c, 1);
digitalWrite(d, 0);
pid_loop();
engage(0);
}

void loop() {
  // put your main code here, to run repeatedly:
 //getYaw();
 //pid_loop();
  }
