#include "orazio_wrapper.h"
#include <cassert>
#include <cstdio>

/* Homework 1 : Wrap the orazio client in a C++ class

   The part of the code to be filled are preceded by a TODO statment

   Example :

   // TODO : some really complicated stuff

   fill here

   //

   other piece of code

*/

struct OrazioClient;

// initializes the member variables to use to read the packets
OrazioWrapper::OrazioWrapper() {
  system_params.header.type = SYSTEM_PARAM_PACKET_ID;
  system_params.header.size = sizeof(SystemParamPacket);

  system_status.header.type = SYSTEM_STATUS_PACKET_ID;
  system_status.header.size = sizeof(SystemStatusPacket);

  drive_status.header.type = DIFFERENTIAL_DRIVE_STATUS_PACKET_ID;
  drive_status.header.size = sizeof(DifferentialDriveStatusPacket);

  sonar_status.header.type = SONAR_STATUS_PACKET_ID;
  sonar_status.header.size = sizeof(SonarStatusPacket);

  drive_control.header.type = DIFFERENTIAL_DRIVE_CONTROL_PACKET_ID;
  drive_control.header.size = sizeof(DifferentialDriveControlPacket);
}

OrazioWrapper::~OrazioWrapper() { disconnect(); }

void OrazioWrapper::connect(const char *serial_device) {
  // 0. create orazio_client object and sync
  // 1. sync
  // 2. query system params
  // 3. set packet mask in system params to update drive, status and sonar
  // 4. send packet mask

  int retries = 50;

  // TODO : 0 Instanciate an orazio client
  PacketStatus control;
  _client = OrazioClient_init (serial_device, 115200);
  //
  
  if (!_client) {
    printf("cannot open client on device [%s]\nABORTING", serial_device);
    return;
  }

  printf("Synching ");
  for (int i = 0; i < retries; ++i) {
    printf(".");
    fflush(stdout);
    
    // TODO : Sync the client
    // which fancy function of Orazio client will you have to call?
    control = OrazioClient_sync(_client, 1);
    //
    
  }
  printf("done\n");

  // TODO : check if the synchronization had succed,
  //        in the negative case disconnect orazio
  if(control < Success)
	disconnect();
	
  //

  // TODO : Try to read the configuration from eeprom,
  //        in the negative case disconnect orazio  
  if (OrazioClient_readConfiguration(_client, retries) < 0)
	disconnect();
  //

  // TODO : Try to get the parameters of the system,
  //        in the negative case disconnect orazio
  if (OrazioClient_get(_client, (PacketHeader*) &system_params) < 0)
    disconnect();
  //

  // TODO : Try to send the new system parameters,
  //        in the negative case disconnect orazio
  if (OrazioClient_sendPacket(_client, (PacketHeader*) &system_params, retries) < 0)
	disconnect();
  //
  _toggleDrive();
}
void OrazioWrapper::_toggleDrive() {
  // activate just the usefull subsystems
  int retries = 10;
  OrazioClient_get(_client, (PacketHeader *)&system_params);
  system_params.header.update_enabled = 1;
  OrazioClient_sendPacket(_client, (PacketHeader *)&system_params, retries);

  ServoParamPacket servo_params;
  servo_params.header.type = SERVO_PARAM_PACKET_ID;
  servo_params.header.size = sizeof(ServoParamPacket);
  OrazioClient_get(_client, (PacketHeader *)&servo_params);
  servo_params.header.update_enabled = 0;
  OrazioClient_sendPacket(_client, (PacketHeader *)&servo_params, retries);

  SonarParamPacket sonar_params;
  sonar_params.header.type = SONAR_PARAM_PACKET_ID;
  sonar_params.header.size = sizeof(SonarParamPacket);

  OrazioClient_get(_client, (PacketHeader *)&sonar_params);
  sonar_params.header.update_enabled = 1;
  OrazioClient_sendPacket(_client, (PacketHeader *)&sonar_params, retries);

  IMUParamPacket imu_params;
  imu_params.header.type = IMU_PARAM_PACKET_ID;
  imu_params.header.size = sizeof(IMUParamPacket);
  OrazioClient_get(_client, (PacketHeader *)&imu_params);
  imu_params.header.update_enabled = 0;
  OrazioClient_sendPacket(_client, (PacketHeader *)&imu_params, retries);

  JointParamPacket joint_params;
  for (int i = 0; i < 4; ++i) {
    joint_params.header.type = JOINT_PARAM_PACKET_ID;
    joint_params.header.size = sizeof(JointParamPacket);
    joint_params.header.index = i;
  }
  OrazioClient_get(_client, (PacketHeader *)&joint_params);
  joint_params.header.update_enabled = 0;
  OrazioClient_sendPacket(_client, (PacketHeader *)&joint_params, retries);

  DifferentialDriveParamPacket drive_params;
  drive_params.header.type = DIFFERENTIAL_DRIVE_PARAM_PACKET_ID;
  drive_params.header.size = sizeof(DifferentialDriveParamPacket);
  OrazioClient_get(_client, (PacketHeader *)&drive_params);
  drive_params.header.update_enabled = 1;
  OrazioClient_sendPacket(_client, (PacketHeader *)&drive_params, retries);
}

void OrazioWrapper::disconnect() {
  // TODO : check if the orazio client is instanciated,
  //        in the positive case destroy it
  if(_client != NULL)
	OrazioClient_destroy(_client);
  //
  _client = 0;
}

bool OrazioWrapper::isConnected() {
  return _client;
  // true if orazio_client not null
}

void OrazioWrapper::getOdometry(float &x, float &y, float &theta) {
  // TODO : get the odometry measure from drive status
  x = drive_status.odom_x;
  y = drive_status.odom_y;
  theta = drive_status.odom_theta;
  //
}

void OrazioWrapper::getVelocities(float &tv, float &rv) {
  // TODO : Read the velocities from drive status
  tv = drive_status.translational_velocity_measured;
  rv = drive_status.rotational_velocity_measured;
  //
}

void OrazioWrapper::setVelocities(float tv, float rv) {
  // TODO : Set velocities of the platform using drive control
  drive_control.translational_velocity = tv;
  drive_control.rotational_velocity = rv;
  OrazioClient_sendPacket(_client, (PacketHeader*) &drive_control, 50);
  //
}

void OrazioWrapper::getSonar(bool &is_new, float &range, int sonar_num) {
  // TODO : Read the ith sonar from the sonar packet
  //        if the epoch of the sonar packet is different from the current epoch
  //        is_new is false and no work have to be done
  //        otherwise assign the new value of the sonar reading to range and set
  //        is_new to true
  range = 0;
  if (sonar_num < 0 || sonar_num >= SONARS_MAX)
    return;
  
  if (sonar_status.header.seq == currentEpoch()){
	  is_new = true;
	  range = sonar_status.ranges[sonar_num];
  }
  //
}

int OrazioWrapper::currentEpoch() {
  // returns the epoch of the last system packet
  return system_status.header.seq;
}

void OrazioWrapper::sync() {
  if (!_client)
    return;
  OrazioClient_sync(_client, 1);
  OrazioClient_get(_client, (PacketHeader *)&system_status);
  OrazioClient_get(_client, (PacketHeader *)&drive_status);
  OrazioClient_get(_client, (PacketHeader *)&sonar_status);
  // if connected, syncs the client
  // updates all status packets
}
