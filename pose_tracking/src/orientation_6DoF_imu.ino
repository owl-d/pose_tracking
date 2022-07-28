#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <Dynamixel2Arduino.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <MPU9250.h>
cMPU9250 mpu;

#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial

///////////////////////////////////////
const uint8_t PERIOD = 20;
double dt = 0.02;
///////////////////////////////////////

const uint8_t DXL_DIR_PIN = 84;//OpenCR Board's DIR PIN.    
const uint8_t RIGHT_ID = 1;
const uint8_t LEFT_ID = 2;
const double DXL_PROTOCOL_VERSION = 2.0;

double set_lin;
double set_ang;
double set_Lvel;
double set_Rvel;
double set_LRPM;
double set_RRPM;

double wheel_wid = 0.16; //turtlebot3_burger's width
double wheel_rad = 0.033; //turtlebot3_burger's radius
double G = 1; // turtlebot3 gear ratio (XL430-W250)
double Re = 4096; // turtlebot3 encoder's resolution (XL430-W250)
double thick_F = (wheel_rad*2*PI)/(G*Re) *10000; //1thick's distance

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem; //This namespace is required to use Control table item names

//Callback function for vel_msg
void twistMessageReceived(const geometry_msgs::Twist& msg)
{
  set_lin = msg.linear.x;
  set_ang = msg.angular.z;
  set_Lvel = set_lin + (set_ang * wheel_wid / 2); //target_speed_l
  set_Rvel = set_lin - (set_ang * wheel_wid / 2); //target_speed_r
  set_LRPM = set_Lvel * 60 / 2 / PI / wheel_rad; //RAW to RPM
  set_RRPM = set_Rvel * 60 / 2 / PI / wheel_rad; //RAW to RPM

  dxl.setGoalVelocity(LEFT_ID, set_LRPM, UNIT_RPM);
  dxl.setGoalVelocity(RIGHT_ID, set_RRPM, UNIT_RPM);
}

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("kalman", &imu_msg);
ros::Subscriber<geometry_msgs::Twist> teleop_sub("cmd_vel",&twistMessageReceived);

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;


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
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  mpu.begin();

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL); 

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000); 
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(RIGHT_ID); 
  dxl.ping(LEFT_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(LEFT_ID);
  dxl.torqueOff(RIGHT_ID);
  dxl.setOperatingMode(LEFT_ID, OP_VELOCITY);
  dxl.setOperatingMode(RIGHT_ID, OP_VELOCITY);
  dxl.torqueOn(LEFT_ID);
  dxl.torqueOn(RIGHT_ID);
  dxl.writeControlTableItem(DRIVE_MODE, LEFT_ID, 0);
  dxl.writeControlTableItem(DRIVE_MODE, RIGHT_ID, 0);
  nh.subscribe(teleop_sub);
}

void loop()
{
  static uint32_t pre_time;

  double gyro[3];
  double acc[3];
  double mag[3];
  double H[12];
  double Q[16];
  double R[9] = {2, 0, 0, 0, 2, 0, 0, 0, 2};
  double V[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double P[16];
  double A[16];
  double K[12];
  double x[4];
  double xp[4];
  double z1[3];
  double z2[3];
  double h[3];

  if (digitalRead(13) == HIGH){
    x[0] = 1; x[1] = 0; x[2] = 0; x[3] = 0;

    P[0]=0.001; P[1]=0; P[2]=0; P[3]=0;
    P[4]=0; P[5]=0.001; P[6]=0; P[7]=0;
    P[8]=0; P[9]=0; P[10]=0.001; P[11]=0;
    P[12]=0; P[13]=0; P[14]=0; P[15]=0.001;

    Q[0]=1e-6; Q[1]=0; Q[2]=0; Q[3]=0;
    Q[4]=0; Q[5]=1e-6; Q[6]=0; Q[7]=0;
    Q[8]=0; Q[9]=0; Q[10]=1e-6; Q[11]=0;
    Q[12]=0; Q[13]=0; Q[14]=0; Q[15]=1e-6;
    
    
    digitalWrite(13, LOW);
  }

  if (millis()-pre_time >= PERIOD)
  {
    pre_time = millis();
    
    mpu.gyro_get_adc();
    mpu.acc_get_adc();
    //mpu.mag_get_adc();
    
    //sensor value
    gyro[0] = mpu.gyroADC[ROLL]*0.0010642;
    gyro[1] = mpu.gyroADC[PITCH]*0.0010642;
    gyro[2] = mpu.gyroADC[YAW]*0.0010642; // *3.14/180/16.4 = *0.00106

    acc[0] = mpu.accADC[ROLL]*0.000598550415;
    acc[1] = mpu.accADC[PITCH]*0.000598550415;
    acc[2] = mpu.accADC[YAW]*0.000598550415; //*9.8/16384
    
//    mag[0] = mpu.magADC[0]*15e-8; //*1200/4096=*0.293
//    mag[1] = mpu.magADC[1]*15e-8;
//    mag[2] = mpu.magADC[2]*15e-8;

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
    H[0] = -2*x[2];
    H[1] = 2*x[3];
    H[2] = -2*x[0];
    H[3] = 2*x[1];
    H[4] = 2*x[1];
    H[5] = 2*x[0];
    H[6] = 2*x[3];
    H[7] = 2*x[2];
    H[8] = 2*x[0];
    H[9] = -2*x[1];
    H[10] = -2*x[2];
    H[11] = 2*x[3];
    
    h[0] = 2*x[1]*x[3] - 2*x[0]*x[2];
    h[1] = 2*x[0]*x[1] + 2*x[2]*x[3];
    h[2] = x[0]*x[0] - x[1]*x[1] - x[2]*x[2] + x[3]*x[3];
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
//    double q1[4] ={ xp[0] + qe1[0], xp[1] + qe1[1], xp[2] + qe1[2], xp[3] + qe1[3]};

    x[0] = xp[0] + qe1[0];
    x[1] = xp[1] + qe1[1];
    x[2] = xp[2] + qe1[2];
    x[3] = xp[3] + qe1[3];

    

    double K_h[16];
    matrix_dot(K_h, K, H, 4, 3, 4);
    double I_Kh[16] = { 1 - K_h[0], -K_h[1], -K_h[2], -K_h[3], -K_h[4], 1 - K_h[5], -K_h[6], -K_h[7], -K_h[8], -K_h[9], 1 - K_h[10], -K_h[11], -K_h[12], -K_h[13], -K_h[14], 1 - K_h[15]};
    
    double P1[16];
//    matrix_dot(P1, I_Kh, Pp, 4, 4, 4);
    matrix_dot(P, I_Kh, Pp, 4, 4, 4);

    
//    //Correction Stage2 : with mag
//    H[0] = 2*x[3];
//    H[1] = 2*x[2];
//    H[2] = 2*x[1];
//    H[3] = 2*x[0];
//    H[4] = 2*x[0];
//    H[5] = -2*x[1];
//    H[6] = -2*x[2];
//    H[7] = -2*x[3];
//    H[8] = -2*x[1];
//    H[9] = -2*x[0];
//    H[10] = 2*x[3];
//    H[11] = 2*x[2];
//        
//    h[0] = 2*x[1]*x[2] + 2*x[0]*x[3];
//    h[1] = x[0]*x[0] - x[1]*x[1] - x[2]*x[2] - x[3]*x[3];
//    h[2] = 2*x[2]*x[3] - 2*x[0]*x[1];
//    
//    z2[0] = mag[0];
//    z2[1] = mag[1];
//    z2[2] = mag[2];
//
//    matrix_dot(Pp_h, H, Pp, 3, 4, 4);
//    double H_T2[12]={ H[0], H[4], H[8], H[1], H[5], H[9], H[2], H[6], H[10], H[3], H[7], H[11]};
//    matrix_dot(Pp_H, Pp_h, H_T2, 3, 4, 3);
//
//    double temp2[9] = {Pp_H[0]+1, Pp_H[1], Pp_H[2], Pp_H[3], Pp_H[4]+1, Pp_H[5], Pp_H[6], Pp_H[7], Pp_H[8]+1};
//    double inverse_temp2[9];
//
//    InverseMatrix3(temp2, inverse_temp2);
//    matrix_dot(H_ti, Pp, H_T2, 4, 4, 3);
//    matrix_dot(K, H_ti, inverse_temp2, 4, 3, 3);
//    
//    double z2_h[3] = {z2[0]-h[0], z2[1]-h[1], z2[2]-h[2]};
//    double qe2[4];
//    matrix_dot(qe2, K, z2_h, 4, 3, 1);
//    qe2[1] = 0;
//    qe2[2] = 0;
//    
//    x[0] = q1[0] + qe2[0];
//    x[1] = q1[1] + qe2[1];
//    x[2] = q1[2] + qe2[2];
//    x[3] = q1[3] + qe2[3];
//    
//    matrix_dot(K_h, K, H, 4, 3, 4);
//    I_Kh[0] = 1 - K_h[0];
//    I_Kh[1] = -K_h[1];
//    I_Kh[2] = -K_h[2];
//    I_Kh[3] = -K_h[3];
//    I_Kh[4] = -K_h[4];
//    I_Kh[5] = 1 - K_h[5];
//    I_Kh[6] = -K_h[6];
//    I_Kh[7] = -K_h[7];
//    I_Kh[8] = -K_h[8];
//    I_Kh[9] = -K_h[9];
//    I_Kh[10] = 1 - K_h[10];
//    I_Kh[11] = -K_h[11];
//    I_Kh[12] = -K_h[12];
//    I_Kh[13] = -K_h[13];
//    I_Kh[14] = -K_h[14];
//    I_Kh[15] = 1 - K_h[15];
//  
//    matrix_dot(P, P1, I_Kh, 4, 4, 4);

    double Q_abs = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
    x[0] = x[0]/Q_abs;
    x[1] = x[1]/Q_abs;
    x[2] = x[2]/Q_abs;
    x[3] = x[3]/Q_abs;

    //Publish
    imu_msg.header.stamp = nh.now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.w = x[0];
    imu_msg.orientation.x = x[1];
    imu_msg.orientation.y = x[2];
    imu_msg.orientation.z = x[3];
    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];
    imu_msg.linear_acceleration.x = acc[0];
    imu_msg.linear_acceleration.y = acc[1];
    imu_msg.linear_acceleration.z = acc[2];

    tfs_msg.header.stamp    = nh.now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id  = "imu_link";
    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.0;
    tfs_msg.transform.rotation.w = imu_msg.orientation.w;
    tfs_msg.transform.rotation.x = imu_msg.orientation.x;
    tfs_msg.transform.rotation.y = imu_msg.orientation.y;
    tfs_msg.transform.rotation.z = imu_msg.orientation.z;

    imu_pub.publish(&imu_msg);
    tfbroadcaster.sendTransform(tfs_msg);
  }

  nh.spinOnce();
}
