/*
  Car driver to control tamiya trunck
  Controll the motor and servo
 */

#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>

#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros_tamiya/Imu.h>

#define STERRING_WHEELS_PIN 8
#define DRIVE_WHEELS_PIN 9
#define LED_PIN 13

Servo sterring_wheels;
Servo drive_wheels;

LSM303 compass;
L3G gyro;

ros::NodeHandle nh;

void callback(const geometry_msgs::Twist& msg){
    setDriverTo(msg.linear.x);
    sterring_wheels.write(msg.angular.z);
    
    digitalWrite(LED_PIN, HIGH-digitalRead(LED_PIN));   // blink the led
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", callback);

ros_tamiya::Imu imu_msg;
ros::Publisher pub("imu", &imu_msg);

// Setup function
void setup() {
  pinMode(LED_PIN, OUTPUT);
  
  sterring_wheels.attach(STERRING_WHEELS_PIN);
  drive_wheels.attach(DRIVE_WHEELS_PIN);
  
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  
  //if (!gyro.init()){ while (1);}
  gyro.init();
  gyro.enableDefault();
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  digitalWrite(LED_PIN, HIGH);
}

// the loop routine runs over and over again forever:
void loop() {
  compass.read();
  imu_msg.heading.data = compass.heading();
  imu_msg.mag.x = compass.m.x;
  imu_msg.mag.y = compass.m.y;
  imu_msg.mag.z = compass.m.z;
  imu_msg.accel.x = compass.a.x;
  imu_msg.accel.y = compass.a.y;
  imu_msg.accel.z = compass.a.z;  

  gyro.read();
  imu_msg.gyro.x = gyro.g.x;
  imu_msg.gyro.y = gyro.g.y;
  imu_msg.gyro.z = gyro.g.z;

  pub.publish(&imu_msg);
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


