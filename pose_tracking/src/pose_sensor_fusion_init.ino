#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <Dynamixel2Arduino.h>
#include <MPU9250.h>

cMPU9250 mpu;

#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial

////////////HYPER PARAM///////////////////////////
const uint8_t PERIOD = 20;
double dt = 0.02;
double Q_PARAM = 1e-6;
/////////////////////////////////////////////////

//Orientation Tracking Param
double gyro[3];
double acc[3];
double mag[3];

double Q[16] = {Q_PARAM, 0, 0, 0, 0, Q_PARAM, 0, 0, 0, 0, Q_PARAM, 0, 0, 0, 0, Q_PARAM};
double P[16] = {0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001};
double R[9] = {2, 0, 0, 0, 2, 0, 0, 0, 2};
double V[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
double A[16];
double K[12];
double H[12];
double h[3];
double z1[3];
double z2[3];
double x[4] = {1.0, 0.0, 0.0, 0.0};
double xp[4];
double theta = 0.0;

//DXL Param
#define WHEEL_NUM       2
#define LEFT            0
#define RIGHT           1

bool init_encoder = true;
int32_t last_diff_tick[2] = {0.0, 0.0};
double  last_rad[2]       = {0.0, 0.0};

const uint8_t DXL_DIR_PIN = 84;//OpenCR Board's DIR PIN.    
const uint8_t RIGHT_ID = 1;
const uint8_t LEFT_ID = 2;
const double DXL_PROTOCOL_VERSION = 2.0;

double L_Vel;
double R_Vel;
double L_RPM;
double R_RPM;
double lin_Vel=0;
double ang_Vel=0;


double cnt_Lpos;
double cnt_Rpos;
double prev_Lpos=0;
double prev_Rpos=0;
double dif_Lpos;
double dif_Rpos;

double yaw = 0;
double dxl_x = 0;
double dxl_y = 0;

double set_lin;
double set_ang;
double set_Lvel;
double set_Rvel;
double set_LRPM;
double set_RRPM;

double WHEEL_SEPARATION = 0.16; //turtlebot3_burger's width
double WHEEL_RADIUS = 0.033; //turtlebot3_burger's radius

#define TICK2RAD   0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem; //This namespace is required to use Control table item names

//Callback function for vel_msg
void twistMessageReceived(const geometry_msgs::Twist& msg)
{
  set_lin = msg.linear.x;
  set_ang = msg.angular.z;
  set_Lvel = set_lin + (set_ang * WHEEL_SEPARATION / 2.0); //target_speed_l
  set_Rvel = set_lin - (set_ang * WHEEL_SEPARATION / 2.0); //target_speed_r
  set_LRPM = set_Lvel * 60.0 / 2.0 / PI / WHEEL_RADIUS; //RAW to RPM
  set_RRPM = set_Rvel * 60.0 / 2.0 / PI / WHEEL_RADIUS; //RAW to RPM

  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
}

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
tf::TransformBroadcaster tfbroadcaster;
std_msgs::Float64 yaw_msg;
geometry_msgs::PoseStamped pose_msg;
geometry_msgs::Vector3 lin_msg;
geometry_msgs::Vector3 ang_msg;
geometry_msgs::TransformStamped tfs_msg;
nav_msgs::Path path_msg;

//ros::Publisher lin_pub("lin_vel", &lin_msg);
//ros::Publisher ang_pub("ang_vel", &ang_msg);
//ros::Publisher mag_pub("magnetic_field", &mag_msg);
//ros::Publisher imu_pub("kalman", &imu_msg);
ros::Publisher pose_pub("pose", &pose_msg);
ros::Publisher yaw_pub("yaw", &yaw_msg);
ros::Publisher path_pub("trajectory", &path_msg); 

ros::Subscriber<geometry_msgs::Twist> teleop_sub("cmd_vel",&twistMessageReceived);

void matrix_dot(double* C, double* A, double* B, int n, int l, int m){
  for(int i = 0; i < n; i++){
    for(int j = 0; j < m; j++){
      *(C+i*m+j) = 0;
      for(int k = 0; k < l; k++)
        *(C+i*m+j) += *(A + i*l + k) * (*(B + j + m*k));
    }
  }
}


int InverseMatrix3(double *m, double *mi){

  double det = m[0] * (m[4] * m[8] - m[7] * m[5]) -m[1] * (m[3] * m[8] - m[5] * m[6]) +m[2] * (m[3] * m[7] - m[4] * m[6]);
  if (det == 0)
    return 0;

  det = 1 / det;

  mi[0] = (m[4] * m[8] - m[7] * m[5]) * det;
  mi[1] = (m[2] * m[7] - m[1] * m[8]) * det;
  mi[2] = (m[1] * m[5] - m[2] * m[4]) * det;
  mi[3] = (m[5] * m[6] - m[3] * m[8]) * det;
  mi[4] = (m[0] * m[8] - m[2] * m[6]) * det;
  mi[5] = (m[3] * m[2] - m[0] * m[5]) * det;
  mi[6] = (m[3] * m[7] - m[6] * m[4]) * det;
  mi[7] = (m[6] * m[1] - m[0] * m[7]) * det;
  mi[8] = (m[0] * m[4] - m[3] * m[1]) * det;
  return 1;

}


void create_A(double* gyro, double* A){
  double dt=0.02;
  A[0] = 1;
  A[1] = -dt/2*gyro[0];
  A[2] = -dt/2*gyro[1];
  A[3] = -dt/2*gyro[2];
  A[4] = dt/2*gyro[0];
  A[5] = 1;
  A[6] = dt/2*gyro[2];
  A[7] = -dt/2*gyro[1];
  A[8] = dt/2*gyro[1];
  A[9] = -dt/2*gyro[2];
  A[10] = 1;
  A[11] = dt/2*gyro[0];
  A[12] = dt/2*gyro[2];
  A[13] = dt/2*gyro[1];
  A[14] = -dt/2*gyro[0];
  A[15] = 1;
}


void setup()
{

  Serial.begin(115200);
  
  nh.initNode();
  
//  nh.advertise(imu_pub);
//  nh.advertise(lin_pub);
//  nh.advertise(ang_pub);
//  nh.advertise(mag_pub);
  nh.advertise(pose_pub);
  nh.advertise(yaw_pub);
  nh.advertise(path_pub);

  
  tfbroadcaster.init(nh);

  mpu.begin();

  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl.ping(RIGHT_ID);
  dxl.ping(LEFT_ID);
  dxl.torqueOff(LEFT_ID);
  dxl.torqueOff(RIGHT_ID);
  dxl.setOperatingMode(LEFT_ID, OP_VELOCITY);
  dxl.setOperatingMode(RIGHT_ID, OP_VELOCITY);
  dxl.torqueOn(LEFT_ID);
  dxl.torqueOn(RIGHT_ID);
  dxl.writeControlTableItem(DRIVE_MODE, LEFT_ID, 0);
  dxl.writeControlTableItem(DRIVE_MODE, RIGHT_ID, 0);
  nh.subscribe(teleop_sub);

  //initial value set
  prev_Lpos = dxl.getPresentPosition(LEFT_ID);
  prev_Rpos = dxl.getPresentPosition(RIGHT_ID);
  dxl_x = 0.0;
  dxl_y = 0.0;

  x[0] = 1.0;
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = 0.0;
  theta = 0.0;
}

void loop()
{
  static uint32_t pre_time;

  if (millis()-pre_time >= PERIOD)
  {
    pre_time = millis();
    orientation_tracking();
    position_tracking();
    publish();
  }
  nh.spinOnce();
}

void position_tracking()
{
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, delta_theta;
  static double last_theta = 0.0;
  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  cnt_Lpos = dxl.getPresentPosition(LEFT_ID);
  cnt_Rpos = dxl.getPresentPosition(RIGHT_ID);
  L_RPM = dxl.getPresentVelocity(LEFT_ID,UNIT_RPM);
  R_RPM = dxl.getPresentVelocity(RIGHT_ID,UNIT_RPM);

  L_Vel = 2.0 * PI * WHEEL_RADIUS * L_RPM / 60.0; //left wheel velocity
  R_Vel = 2.0 * PI * WHEEL_RADIUS * R_RPM / 60.0; //right wheel velocity

  lin_Vel = (L_Vel + R_Vel) /2.0; //linear_velocity
  ang_Vel = (R_Vel - L_Vel) /0.16; //angular_velocity
 
  dif_Lpos = cnt_Lpos - prev_Lpos; //diff_l
  dif_Rpos = cnt_Rpos - prev_Rpos; //diff_r

  wheel_l = TICK2RAD * dif_Lpos;
  wheel_r = TICK2RAD * dif_Rpos;

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;
    
  delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  
  // compute pose
  dxl_x += delta_s * cos(theta);
  dxl_y += delta_s * sin(theta);

  prev_Lpos = cnt_Lpos;
  prev_Rpos = cnt_Rpos;
}

void orientation_tracking()
{  
  mpu.gyro_get_adc();
  mpu.acc_get_adc();
  mpu.mag_get_adc();
  
  //sensor value
  gyro[0] = mpu.gyroADC[ROLL]*0.0010642;
  gyro[1] = mpu.gyroADC[PITCH]*0.0010642;
  gyro[2] = mpu.gyroADC[YAW]*0.0010642; // *3.14/180/16.4 = *0.00106

  acc[0] = mpu.accADC[ROLL]*0.000598550415;
  acc[1] = mpu.accADC[PITCH]*0.000598550415;
  acc[2] = mpu.accADC[YAW]*0.000598550415; //*9.8/16384
  
  mag[0] = mpu.magADC[0]*15e-8; //*1200/4096=*0.293
  mag[1] = mpu.magADC[1]*15e-8;
  mag[2] = mpu.magADC[2]*15e-8;

  //Priori System Estimate : gyro
  create_A(gyro, A);
  matrix_dot(xp, A, x, 4, 4, 1);
  double Pp_a[16];
  matrix_dot(Pp_a, A, P, 4, 4, 4);
  double A_T[16] = {A[0], A[4], A[8], A[12], A[1], A[5], A[9], A[13], A[2], A[6], A[10], A[14], A[3], A[7], A[11], A[15]};
  double Pp_A[16];
  matrix_dot(Pp_A, Pp_a, A_T, 4, 4, 4);
  double Pp[16] = {Pp_A[0]+Q[0], Pp_A[1]+Q[1], Pp_A[2]+Q[2], Pp_A[3]+Q[3], Pp_A[4]+Q[4], Pp_A[5]+Q[5], Pp_A[6]+Q[6], Pp_A[7]+Q[7], Pp_A[8]+Q[8], Pp_A[9]+Q[9], Pp_A[10]+Q[10], Pp_A[11]+Q[11], Pp_A[12]+Q[12], Pp_A[13]+Q[13], Pp_A[14]+Q[14], Pp_A[15]+Q[15]};

  //Correction Stage1 : with acc
  H[0] = -2*xp[2];
  H[1] = 2*xp[3];
  H[2] = -2*xp[0];
  H[3] = 2*xp[1];
  H[4] = 2*xp[1];
  H[5] = 2*xp[0];
  H[6] = 2*xp[3];
  H[7] = 2*xp[2];
  H[8] = 2*xp[0];
  H[9] = -2*xp[1];
  H[10] = -2*xp[2];
  H[11] = 2*xp[3];
  
  h[0] = 2*xp[1]*xp[3] - 2*xp[0]*xp[2];
  h[1] = 2*xp[0]*xp[1] + 2*xp[2]*xp[3];
  h[2] = xp[0]*xp[0] - xp[1]*xp[1] - xp[2]*xp[2] + xp[3]*xp[3];
  h[0] = h[0]*9.8;
  h[1] = h[1]*9.8;
  h[2] = h[2]*9.8;
  
  z1[0] = acc[0];
  z1[1] = acc[1];
  z1[2] = acc[2];

  //Kalman gain
  double H_T[12]={ H[0], H[4], H[8], H[1], H[5], H[9], H[2], H[6], H[10], H[3], H[7], H[11]};
  double Pp_h[12];
  matrix_dot(Pp_h, H, Pp, 3, 4, 4);
  double Pp_H[9];
  matrix_dot(Pp_H, Pp_h, H_T, 3, 4, 3);

  double R_v[9];
  matrix_dot(R_v, V, R, 3, 3, 3);
  double R_V[9];
  matrix_dot(R_V, R_v, V, 3, 3, 3);
  
  double temp[9] = {Pp_H[0]+R_V[0], Pp_H[1]+R_V[1], Pp_H[2]+R_V[2], Pp_H[3]+R_V[3], Pp_H[4]+R_V[4], Pp_H[5]+R_V[5], Pp_H[6]+R_V[6], Pp_H[7]+R_V[7], Pp_H[8]+R_V[8]};
  
  double inverse_temp[9];
  InverseMatrix3(temp, inverse_temp);
  
  double H_ti[12];
  matrix_dot(H_ti, Pp, H_T, 4, 4, 3);
  matrix_dot(K, H_ti, inverse_temp, 4, 3, 3);
  
  double z1_h[3] = {z1[0]-h[0], z1[1]-h[1], z1[2]-h[2]};
  double qe1[4];
  matrix_dot(qe1, K, z1_h, 4, 3, 1);
  qe1[3] = 0;
  double q1[4] ={ xp[0] + qe1[0], xp[1] + qe1[1], xp[2] + qe1[2], xp[3] + qe1[3]};

  double K_h[16];
  matrix_dot(K_h, K, H, 4, 3, 4);
  double I_Kh[16] = { 1 - K_h[0], -K_h[1], -K_h[2], -K_h[3], -K_h[4], 1 - K_h[5], -K_h[6], -K_h[7], -K_h[8], -K_h[9], 1 - K_h[10], -K_h[11], -K_h[12], -K_h[13], -K_h[14], 1 - K_h[15]};
  
  double P1[16];
  matrix_dot(P1, I_Kh, Pp, 4, 4, 4);
  //matrix_dot(P, I_Kh, Pp, 4, 4, 4);

  
  //Correction Stage2 : with mag
  H[0] = 2*xp[3];
  H[1] = 2*xp[2];
  H[2] = 2*xp[1];
  H[3] = 2*xp[0];
  H[4] = 2*xp[0];
  H[5] = -2*xp[1];
  H[6] = -2*xp[2];
  H[7] = -2*xp[3];
  H[8] = -2*xp[1];
  H[9] = -2*xp[0];
  H[10] = 2*xp[3];
  H[11] = 2*xp[2];
      
  h[0] = 2*xp[1]*xp[2] + 2*xp[0]*xp[3];
  h[1] = xp[0]*xp[0] - xp[1]*xp[1] - xp[2]*xp[2] - xp[3]*xp[3];
  h[2] = 2*xp[2]*xp[3] - 2*xp[0]*xp[1];
  
  z2[0] = mag[0];
  z2[1] = mag[1];
  z2[2] = mag[2];

  matrix_dot(Pp_h, H, Pp, 3, 4, 4);
  double H_T2[12]={ H[0], H[4], H[8], H[1], H[5], H[9], H[2], H[6], H[10], H[3], H[7], H[11]};
  matrix_dot(Pp_H, Pp_h, H_T2, 3, 4, 3);

  double temp2[9] = {Pp_H[0]+1, Pp_H[1], Pp_H[2], Pp_H[3], Pp_H[4]+1, Pp_H[5], Pp_H[6], Pp_H[7], Pp_H[8]+1};
  double inverse_temp2[9];

  InverseMatrix3(temp2, inverse_temp2);
  matrix_dot(H_ti, Pp, H_T2, 4, 4, 3);
  matrix_dot(K, H_ti, inverse_temp2, 4, 3, 3);
  
  double z2_h[3] = {z2[0]-h[0], z2[1]-h[1], z2[2]-h[2]};
  double qe2[4];
  matrix_dot(qe2, K, z2_h, 4, 3, 1);
  qe2[1] = 0;
  qe2[2] = 0;
  
  x[0] = q1[0] + qe2[0];
  x[1] = q1[1] + qe2[1];
  x[2] = q1[2] + qe2[2];
  x[3] = q1[3] + qe2[3];
  
  matrix_dot(K_h, K, H, 4, 3, 4);
  I_Kh[0] = 1.0 - K_h[0];
  I_Kh[1] = -K_h[1];
  I_Kh[2] = -K_h[2];
  I_Kh[3] = -K_h[3];
  I_Kh[4] = -K_h[4];
  I_Kh[5] = 1.0 - K_h[5];
  I_Kh[6] = -K_h[6];
  I_Kh[7] = -K_h[7];
  I_Kh[8] = -K_h[8];
  I_Kh[9] = -K_h[9];
  I_Kh[10] = 1.0 - K_h[10];
  I_Kh[11] = -K_h[11];
  I_Kh[12] = -K_h[12];
  I_Kh[13] = -K_h[13];
  I_Kh[14] = -K_h[14];
  I_Kh[15] = 1.0 - K_h[15];

  matrix_dot(P, I_Kh, P1, 4, 4, 4);

  double Q_abs = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
  x[0] = x[0]/Q_abs; //q.x
  x[1] = x[1]/Q_abs; //q.y
  x[2] = x[2]/Q_abs; //q.z
  x[3] = x[3]/Q_abs; //q.w

  //IMU position estimation
  double siny_cosp = 2.0 * (x[0] * x[3] + x[1] * x[2]);
  double cosy_cosp = 1.0 - 2.0 * (x[2] * x[2] + x[3] * x[3]);
  
  theta = atan2(siny_cosp, cosy_cosp); // yaw (z-axis rotation)
  //theta = atan2(2*(x[3]*x[0] + x[1]*x[2]), x[1]*x[1] - x[2]*x[2] - x[3]*x[3] + x[0]*x[0]);
}

void publish()
{
//  imu_msg.header.stamp = nh.now();
//  imu_msg.header.frame_id = "imu_link";
//  imu_msg.orientation.w = x[0];
//  imu_msg.orientation.x = x[1];
//  imu_msg.orientation.y = x[2];
//  imu_msg.orientation.z = x[3];
//  imu_msg.angular_velocity.x = gyro[0];
//  imu_msg.angular_velocity.y = gyro[1];
//  imu_msg.angular_velocity.z = gyro[2];
//  imu_msg.linear_acceleration.x = acc[0];
//  imu_msg.linear_acceleration.y = acc[1];
//  imu_msg.linear_acceleration.z = acc[2];
//
//  mag_msg.header.stamp = nh.now();
//  mag_msg.header.frame_id = "mag_link";
//  mag_msg.magnetic_field.x = mag[0];
//  mag_msg.magnetic_field.y = mag[1];
//  mag_msg.magnetic_field.z = mag[2];

  
  yaw_msg.data = theta*180/PI; //degree

  pose_msg.header.stamp = nh.now();
  pose_msg.header.frame_id = "turtlebot_link";
  pose_msg.pose.position.x = dxl_x;
  pose_msg.pose.position.y = dxl_y;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation.w = x[0];
  pose_msg.pose.orientation.x = x[1];
  pose_msg.pose.orientation.y = x[2];
  pose_msg.pose.orientation.z = x[3];

  tfs_msg.header.stamp    = nh.now();
  tfs_msg.header.frame_id = "map";
  tfs_msg.child_frame_id  = "turtlebot_link";
  
  tfs_msg.transform.translation.x = pose_msg.pose.position.x;
  tfs_msg.transform.translation.y = pose_msg.pose.position.y;
  tfs_msg.transform.translation.z = 0.0;
  tfs_msg.transform.rotation.w = pose_msg.pose.orientation.w;
  tfs_msg.transform.rotation.x = pose_msg.pose.orientation.x;
  tfs_msg.transform.rotation.y = pose_msg.pose.orientation.y;
  tfs_msg.transform.rotation.z = pose_msg.pose.orientation.z;

  path_msg.header.stamp = nh.now();
  path_msg.header.frame_id = "turtlebot_link";
  path_msg.poses.push_back(**pose_msg);
  
//  imu_pub.publish(&imu_msg);
//  mag_pub.publish(&mag_msg);
  pose_pub.publish(&pose_msg);
  yaw_pub.publish(&yaw_msg);
  path_pub.publish(path_msg);
  tfbroadcaster.sendTransform(tfs_msg); 
}
