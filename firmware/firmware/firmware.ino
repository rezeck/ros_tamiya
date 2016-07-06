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
#include <avr/wdt.h>

#include <ros.h>
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <ros_tamiya/Imu.h>
#include <std_msgs/Float32.h>
#include <ros_tamiya/Motor.h>
/********************************************************************/

/********************************************************************/
/*  Arduino PIN Setup                                               */
/********************************************************************/
#define STERRING_WHEELS_PIN 8
#define DRIVE_WHEELS_PIN 9
#define LED_PIN 13
#define GREEN_LED_PIN 3
#define YELLOW_LED_PIN 13
#define CHANNEL_1 10
#define CHANNEL_2 11
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
int neutral = 1600;
const float alpha = 0.6;
float fxm = 0;
float fym = 0;
float fzm = 0;

int channel_1;
int channel_2;
int channel1_default;
int channel2_default;
int timer = 0;
int controller_tresh_min = 300;
int controller_tresh_max = 700;

unsigned long prev_millis;
byte MAX_CONTROL_DELAY = 2;
boolean IS_STOP = false;
/********************************************************************/

static inline unsigned long elapsed() { return millis(); }

/********************************************************************/
/*  ROS Callbacks                                                   */
/********************************************************************/
//void callback(const geometry_msgs::Twist& msg){
void callback(const geometry_msgs::Vector3& msg){
    prev_millis = elapsed();
    if(!IS_STOP){
      setDriverTo(msg.x);
      setSterringTo(msg.z);
      digitalWrite(GREEN_LED_PIN, HIGH);
      delay(10);
      digitalWrite(GREEN_LED_PIN, LOW);
    }
}
/********************************************************************/

float checkElapsedSeconds(){
  unsigned long current_millis;
  current_millis = elapsed();
  return (current_millis - prev_millis) / 1000;
}

void stopVehicle(){
  setDriverTo(300);
  setSterringTo(0);
}

/********************************************************************/
/*  ROS Setup                                                       */
/********************************************************************/
ros::NodeHandle_<ArduinoHardware, 6, 6, 150, 150> nh; 
ros::Subscriber<geometry_msgs::Vector3> sub("cmd_vel", callback);

ros_tamiya::Imu imu_msg;
//ros::Publisher pub_imu("imu", &imu_msg);
std_msgs::Float32 imu_heading;
ros::Publisher pub_heading("heading", &imu_heading);

//ros_tamiya::Motor motor_msg;
//ros::Publisher pub_motor("motor_position", &motor_msg);
/********************************************************************/

/********************************************************************/
/*  Arduino Setup function                                          */
/********************************************************************/
void setup() {
  wdt_enable(WDTO_2S);
  wdt_reset();
  
  Serial.begin(9600);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);
    
  pinMode(CHANNEL_1, INPUT);
  pinMode(CHANNEL_2, INPUT);
  channel1_default = pulseIn(CHANNEL_1, HIGH, 75000);
  channel2_default = pulseIn(CHANNEL_2, HIGH, 75000);
  if(channel1_default < 1300 || channel2_default < 1300){
    channel1_default = 0;
    channel2_default = 0;
    controller_tresh_min = 1300;
    controller_tresh_max = 1700;
  }
  
  /******************************************************/  
  /* Setup motor drive and sterring
  /******************************************************/
  ServoNothing.attach(3);
  
  sterring_wheels.attach(STERRING_WHEELS_PIN);
  drive_wheels.attach(DRIVE_WHEELS_PIN);
  drive_wheels.writeMicroseconds(neutral); // Neutral
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
//  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(pub_motor);
  //nh.advertise(pub_imu);
  nh.advertise(pub_heading);
  /******************************************************/
  
  delay(50);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  
}
/********************************************************************/

/********************************************************************/
/* Arduino loop function                                            */
/********************************************************************/
void loop() {
    wdt_reset();
    
    channel_1 = pulseIn(CHANNEL_1, HIGH, 75000); // Read the pulse width of 
    channel_2 = pulseIn(CHANNEL_2, HIGH, 75000); // each channel
    int a = abs(channel_1 - channel1_default);
    int b = abs(channel_2 - channel2_default);
    
    if ((a > controller_tresh_min && a < controller_tresh_max) || (b > controller_tresh_min && b < controller_tresh_max)){
      IS_STOP = true;
      stopVehicle();
    }
    else{
      IS_STOP = false;
    }
    
    if(checkElapsedSeconds() >= MAX_CONTROL_DELAY){
      stopVehicle();
    }
    
    wdt_reset();
    nh.spinOnce();

    getMagnetometerMsg();
    getAccelerometerMsg();
    
    getTemperatureMsg();  
    getPressureMsg();
    getAltitudeMsg();
    getGyroscopeMsg();
    
    //Serial.println(imu_msg.heading);
    
    //motor_msg.motor_position = drive_wheels.read();
    //motor_msg.sterring_position = sterring_wheels.read();
    
    //pub_motor.publish(&motor_msg);
    pub_heading.publish(&imu_heading);
    //pub_imu.publish(&imu_msg);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    delay(10);
    digitalWrite(YELLOW_LED_PIN, LOW);
}
/********************************************************************/

/********************************************************************/
/* Read magnetometer datas from sensor set it value to ros message  */
/********************************************************************/
void getMagnetometerMsg(){
  compass.read();
  float compass_heading, xm_off, ym_off, zm_off, xm_cal, ym_cal, zm_cal, fxm_comp, fym_comp;
  
  // Magnetometer calibration
  // Tutorial used: http://forum.arduino.cc/index.php?topic=265541.0
  xm_off = compass.m.x * (100000.0/1100.0) -  (137.195048);
  ym_off = compass.m.y * (100000.0/1100.0) -  (-776.479788);
  zm_off = compass.m.z * (100000.0/980.0)  -  (809.501242);
  xm_cal = (0.994089      * xm_off) + (0.013603 * ym_off) + ((-0.005624)  * zm_off);
  ym_cal = (0.013603      * xm_off) + (0.970578 * ym_off) + (0.018167     * zm_off);
  zm_cal = ((-0.005624)   * xm_off) + (0.018167 * ym_off) + (0.976481     * zm_off);
  
  // Low-Pass filter magnetometer
  fxm = xm_cal * alpha + (fxm * (1.0 - alpha));
  fym = ym_cal * alpha + (fym * (1.0 - alpha));
  fzm = zm_cal * alpha + (fzm * (1.0 - alpha));
  
  // Arctangent of y/x
  compass_heading = (atan2(fym,fxm) * 180.0)/M_PI;
  if (compass_heading < 0) compass_heading += 360;
  if(compass_heading < 0) compass_heading += 360;
  
  compass_heading = 360 - compass_heading;
  
  imu_msg.heading = compass_heading;
  imu_heading.data = compass_heading;
  imu_msg.magnetometer.x = fxm;
  imu_msg.magnetometer.y = fym;
  imu_msg.magnetometer.z = fzm;
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
/* Read ros commands and control drive                              */
/********************************************************************/
void setDriverTo(int velocity){ // 1000:2000 (forward:backward)
  if(velocity > 300) velocity = 300;
  else if (velocity < 50) velocity = 50;

  int calcVel = abs(velocity + 300) + 1000;
  if (calcVel == drive_wheels.readMicroseconds()) return;
  
  drive_wheels.writeMicroseconds(calcVel); // set velocity
}
/********************************************************************/

/********************************************************************/
/* Read ros commands and control sterring                           */
/********************************************************************/
void setSterringTo(int angle){ // 0:190 (left:right)
  if(angle > 95) angle = 95;
  else if (angle < -95) angle = -95;

  int calcAng = angle + 95;
  if (calcAng == sterring_wheels.read()) return;
  
  sterring_wheels.write(calcAng); // set angle
}
/********************************************************************/

/********************************************************************/
/* END OF CODE                                                      */
/********************************************************************/

