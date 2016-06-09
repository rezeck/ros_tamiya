/********************************************************************/
/*            Driver to control tamiya txt-1 with ROS
/*         Computer Vision and Robotic Laboratory (VeRlab)
/********************************************************************/

/********************************************************************/
/*  Libraries                                                       */
/********************************************************************/
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <SFE_BMP180.h>

#include <Servo.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros_tamiya/Imu.h>
#include <ros_tamiya/Motor.h>
/********************************************************************/

/********************************************************************/
/*  Arduino PIN Setup                                               */
/********************************************************************/
#define STERRING_WHEELS_PIN 8
#define DRIVE_WHEELS_PIN 9
#define LED_PIN 13
#define GREEN_LED_PIN 3
/********************************************************************/

/********************************************************************/
/*  Main objects and global variables                               */
/********************************************************************/
Servo ServoNothing;
Servo sterring_wheels;
Servo drive_wheels;

SFE_BMP180 pressure;
LSM303 compass;
L3G gyro;

double baseline;
int neutral = 102;
/********************************************************************/

/********************************************************************/
/*  ROS Callbacks                                                   */
/********************************************************************/
void callback(const geometry_msgs::Twist& msg){
    setDriverTo(msg.linear.x);
    sterring_wheels.write(msg.angular.z);
    
    digitalWrite(GREEN_LED_PIN, HIGH);
    delay(20);
    digitalWrite(GREEN_LED_PIN, LOW);
}
/********************************************************************/

/********************************************************************/
/*  ROS Setup                                                       */
/********************************************************************/
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", callback);

ros_tamiya::Imu imu_msg;
ros::Publisher pub_imu("imu", &imu_msg);

ros_tamiya::Motor motor_msg;
ros::Publisher pub_motor("motor_position", &motor_msg);
/********************************************************************/

/********************************************************************/
/*  Arduino Setup function                                          */
/********************************************************************/
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);

  /******************************************************/  
  /* Setup motor drive and sterring
  /******************************************************/
  ServoNothing.attach(3);
  
  sterring_wheels.attach(STERRING_WHEELS_PIN);
  drive_wheels.attach(DRIVE_WHEELS_PIN);
  drive_wheels.write(neutral); // Neutral
  sterring_wheels.write(95); // Neutral
  
  /******************************************************/
  
  /******************************************************/
  /* Setup IMU sensor
  /******************************************************/
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  
  pressure.begin();
  
  // Get baseline to compute altitude
  getTemperatureMsg();  
  getPressureMsg();
  baseline = imu_msg.pressure;
  
  gyro.init();
  gyro.enableDefault();
  /******************************************************/
  
  /******************************************************/
  /* Setup ROS
  /******************************************************/
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_motor);
  nh.advertise(pub_imu);
  /******************************************************/
  
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  
}
/********************************************************************/

/********************************************************************/
/* Arduino loop function                                            */
/********************************************************************/
void loop() {
    nh.spinOnce();

    getMagnetometerMsg();
    getAccelerometerMsg();
    nh.spinOnce();
    
    getTemperatureMsg();  
    getPressureMsg();
    getAltitudeMsg();
    getGyroscopeMsg();
    nh.spinOnce();
    
    motor_msg.motor_position = drive_wheels.read();
    motor_msg.sterring_position = sterring_wheels.read();
    
    pub_motor.publish(&motor_msg);
    pub_imu.publish(&imu_msg);
    delay(10);
}
/********************************************************************/

/********************************************************************/
/* Read magnetometer datas from sensor set it value to ros message  */
/********************************************************************/
void getMagnetometerMsg(){
  compass.read();
  imu_msg.heading = compass.heading();
  imu_msg.magnetometer.x = compass.m.x;
  imu_msg.magnetometer.y = compass.m.y;
  imu_msg.magnetometer.z = compass.m.z;
}
/********************************************************************/

/********************************************************************/
/* Read accelerometer datas from sensor set it value to ros message */
/********************************************************************/
void getAccelerometerMsg(){
  compass.read();
  imu_msg.accelerometer.x = compass.a.x;
  imu_msg.accelerometer.y = compass.a.y;
  imu_msg.accelerometer.z = compass.a.z;
}
/********************************************************************/

/********************************************************************/
/* Read gyroscope datas from sensor set it value to ros message     */
/********************************************************************/
void getGyroscopeMsg(){
  gyro.read();
  imu_msg.gyroscope.x = gyro.g.x;
  imu_msg.gyroscope.y = gyro.g.y;
  imu_msg.gyroscope.z = gyro.g.z;
}
/********************************************************************/

/********************************************************************/
/* Read temperature from sensor set it value to ros message         */
/********************************************************************/
void getTemperatureMsg(){
  double T;
  pressure.startTemperature();
  pressure.getTemperature(T);
  imu_msg.temperature = T;
}
/********************************************************************/

/********************************************************************/
/* Read pressure from sensor set it value to ros message            */
/********************************************************************/
void getPressureMsg(){
  double T, P;
  T = (double)imu_msg.temperature;
  pressure.startPressure(3);
  pressure.getPressure(P, T);
  imu_msg.pressure = (float)P;
}
/********************************************************************/

/********************************************************************/
/* Read altitude from sensor set it value to ros message            */
/********************************************************************/
void getAltitudeMsg(){
    imu_msg.altitude = (float)pressure.altitude((double)imu_msg.pressure, baseline);
}
/********************************************************************/

/********************************************************************/
/* Read ros commands and control drive and sterring                 */
/********************************************************************/
void setDriverTo(int velocity){ // 1000:2000 (forward:backward)
  if (velocity == drive_wheels.read()) return;
  
  if (velocity < 0) velocity = neutral;
  else if (velocity > 180) velocity = neutral;

  bool dir = drive_wheels.read() <= neutral;
  bool newdir = velocity <= neutral;

  if (dir != newdir){
    drive_wheels.write(velocity); // set velocity
    delay(150);
    drive_wheels.write(neutral); // set to Neutral
    //nh.spinOnce();
    delay(150);
  }
  drive_wheels.write(velocity); // set velocity

}
/********************************************************************/

/********************************************************************/
/* END OF CODE                                                      */
/********************************************************************/

