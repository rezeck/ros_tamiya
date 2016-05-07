/*
  Car driver to control tamiya trunck
  Controll the motor and servo
 */

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define STERRING_WHEELS_PIN 8
#define DRIVE_WHEELS_PIN 9

Servo sterring_wheels;
Servo drive_wheels;

ros::NodeHandle nh;

void callback(const geometry_msgs::Twist& msg){
    setDriverTo(msg.linear.x);
    sterring_wheels.write(msg.angular.z);
    
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", callback);


void setup() {
  pinMode(13, OUTPUT);
  sterring_wheels.attach(STERRING_WHEELS_PIN);
  drive_wheels.attach(DRIVE_WHEELS_PIN);

  nh.initNode();
  nh.subscribe(sub);
}

// the loop routine runs over and over again forever:
void loop() {
  nh.spinOnce();
  delay(500);
}

void setDriverTo(int v){
	if (v > 180 || v < 0) return;

        int pos = drive_wheels.read();

	int i = (v-pos)/abs(v-pos);

	while (drive_wheels.read() != v){
	    drive_wheels.write(drive_wheels.read() + i);
    	    delayMicroseconds(5000);
	}
}


