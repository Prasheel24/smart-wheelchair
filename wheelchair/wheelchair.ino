#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <SoftwareSerial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

SoftwareSerial SWSerial(10, 11); // RX on no pin (unused), TX on pin 11 (to S1).

int linear=0;
float angular=0;

ros::NodeHandle node;

void CommandCallBack(const geometry_msgs::Twist &msg)
{
  linear=msg.linear.x;
  linear=map(linear,-1,1,-1000,1000);
  angular=msg.angular.z;
}
std_msgs::String str_msg;
ros::Subscriber<geometry_msgs::Twist> sub("/wheelchair/mobile_base_controller/cmd_vel", &CommandCallBack);
ros::Publisher pub("Ack", &str_msg);

void setup() {    
  node.initNode();
  node.subscribe(sub);  
  node.advertise(pub);
  pinMode(13, OUTPUT);  
  SWSerial.begin(9600);  
}

void loop() {
  if (node.connected())
  {
  str_msg.data = "Throttle";
  SWSerial.print("M1:");
  SWSerial.println(linear);
  SWSerial.print("M2:");
  SWSerial.println(linear);
  digitalWrite(13, HIGH);  
  delay(50);               
  digitalWrite(13, LOW);   
  pub.publish(&str_msg);
  }
  node.spinOnce();
  delay(100);
}



