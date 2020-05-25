/************************************************************************
BSD 3-Clause License
Copyright (c) 2020, Raj Shinde
Copyright (c) 2020, Prasheel Renkuntla
Copyright (c) 2020, Shubham Sonawane
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

/**
 *  @copyright BSD 3-Clause License 
 *  @copyright Copyright © 2020 Raj Shinde, Prasheel Renkuntla, Shubham Sonawane
 *  @file    wheelchair_navigation.ino
 *  @author  Raj Shinde
 *  @date    22/05/2020
 *  @version 0.1
 *  @brief   Autonomous Wheelchair
 *  @section File to control motor speed
 */
#include<Servo.h>
//#include <ros.h>
//#include <geometry_msgs/Twist.h>
//#include <SoftwareSerial.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>

//double linear = 0;
//double angular = 0;
//double rpm1 = 0;
//double rpm2 = 0;
//const axil = ;
//const radius = ;
//ros::NodeHandle node;
//std_msgs::String str_msg;
//ros::Subscriber<geometry_msgs::Twist> sub("/wheelchair/mobile_base_controller/cmd_vel", &CommandCallBack);
//ros::Publisher pub("Ack", &str_msg);
const int encA1 = 2;
const int encB1 = 3; 
const int encA2 = 19;
const int encB2 = 18; 
const double ppr = 7;
int total1 = 0;
int total2 = 0;
double kp = 2;
double ki = 1;
double kd = 2;
double errorSum1 = 0;
double errorSum2 = 0;
double prevError1 = 0;
double prevError2 = 0;
double targetRpm1 = 0;
double targetRpm2 = 0;
double tick1 = 0;
double tick2 = 0;
int dir1 = 0;
int dir2 = 0;
double rpm1 = 0;
double rpm2 = 0;
Servo sb1;
Servo sb2;

void CommandCallBack(const geometry_msgs::Twist &msg) {
  linear=msg.linear.x;
  angular=msg.angular.z;
  vb= angular * bodyRadius;
  v1=v1-vb;
  v2=v2+vb;
}

void setup() {
//  node.initNode();
//  node.subscribe(sub);  
//  node.advertise(pub);
  sb1.attach(5);
  sb2.attach(6);
  sb1.writeMicroseconds(1500);
  sb2.writeMicroseconds(1500);
  pinMode(encA1,INPUT_PULLUP);
  pinMode(encB1,INPUT_PULLUP);
  pinMode(encA2,INPUT_PULLUP);
  pinMode(encB2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA1), encoderCallback1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), encoderCallback2, RISING);
  Serial.begin(115200);
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

void loop() {
//  if (node.connected()) {
//    str_msg.data = "Throttle";
//    SWSerial.print("M1:");
//    SWSerial.println(linear);
//    SWSerial.print("M2:");
//    SWSerial.println(linear);
//    digitalWrite(13, HIGH);  
//    delay(50);               
//    digitalWrite(13, LOW);   
//    pub.publish(&str_msg);
//  }
//  node.spinOnce();
//  delay(100);

  targetRpm2 = 70;
  Serial.println("set 70");
  delay(5000);
  targetRpm2 = 50;
  Serial.println("set 50");
  delay(5000);
  targetRpm2 = 30;
  Serial.println("set 30");
  delay(5000);
  targetRpm2 = -30;
  Serial.println("set -30");
  delay(5000);
  targetRpm2 = -50;
  Serial.println("set -50");
  delay(5000);
  targetRpm2 = -70;
  Serial.println("set -70");
  delay(5000);
  
}

void encoderCallback1() {
  tick1 += 1;   // Increment encoder count on interrupt
  total1 += 1;
  dir1 =  digitalRead(encB1);
  if (dir1 == 0) 
    dir1 = -1;
//  Serial.print(dir1);
}

void encoderCallback2() {
  tick2 += 1;   // Increment encoder count on interrupt
  total2 += 1;
  dir2 =  digitalRead(encB2);
  if (dir2 == 0) 
    dir2 = 1;
  else 
    dir2=-1;
//  Serial.print(dir2);
}

int computePid1(const double &currentRpm, const double &setpoint) {
  double error = setpoint - currentRpm;
  int processValue = kp*error + ki*errorSum1 + kd*(error-prevError1);
  errorSum1 += error;
  prevError1 = error;
  if (errorSum1 > 2000) errorSum1 = 2000;
  if (errorSum1 < 1000) errorSum1 = 1000;
  return processValue;
}

int computePid2(const double &currentRpm, const double &setpoint) {
  double error = setpoint - currentRpm;
  int processValue = kp*error + ki*errorSum2 + kd*(error-prevError2);
  errorSum2 += error;
  prevError2 = error;
  if (errorSum2 >2000) errorSum2 = 2000;
  if (errorSum2 <1000) errorSum2 = 1000;
  return processValue;
}

ISR(TIMER1_OVF_vect)        // interrupt service routine - tick every 0.1sec
{
   TCNT1 = 59286;
//  Serial.println("Encoder");
//  Serial.println(total);
  
  double revolution1 = tick1/ppr;
  double currentRpm1 = 600*revolution1*dir1;
  double revolution2 = tick2/ppr;
  double currentRpm2 = 600*revolution2*dir2;
  
  Serial.print("currentRpm 1: ");
  Serial.print(currentRpm1/71);
  Serial.print(" ");
  Serial.print("targetRpm 1: ");
  Serial.println(targetRpm1);

  Serial.print("currentRpm 2: ");
  Serial.print(currentRpm2/71);
  Serial.print(" ");
  Serial.print("targetRpm 2: ");
  Serial.println(targetRpm2);
  
  tick1 = 0;                // reset encoder counts
  tick2 = 0;                // reset encoder counts
  int pwm1=computePid1((currentRpm1/71), targetRpm1);
  int pwm2=computePid2((currentRpm2/71), targetRpm2);
  
  Serial.print("PWM 1: ");
  Serial.println(pwm1);
  
  Serial.print("PWM 2: ");
  Serial.println(pwm2);

  if (targetRpm2 == 0) {
    sb2.writeMicroseconds(1500);
  }
  else if (pwm2 < 2000 && pwm2 >1000) {
    sb2.writeMicroseconds(pwm2);
  }
  else{
    if (pwm2>2000) {
      sb2.writeMicroseconds(2000);
    }
    else if (pwm2<1000) {
      sb2.writeMicroseconds(1000);
    }
  }
}


