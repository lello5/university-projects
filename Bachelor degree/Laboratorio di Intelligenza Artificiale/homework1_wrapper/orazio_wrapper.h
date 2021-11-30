#include "orazio_client.h"
#include "orazio_packets.h"

struct OrazioClient;

class OrazioWrapper {
public:
  OrazioWrapper();
  OrazioWrapper(const OrazioWrapper &) =
      delete; // we disable the default copy ctor
  ~OrazioWrapper();
  // connects to the base, in case of faiolure the _client is set to 0;
  void connect(const char *serial_device = "/dev/ttyACM0");

  // destroyes the client
  void disconnect();

  // true if the _client is valid
  bool isConnected();

  // reads odometry from internal variable, updated on sync (drive_staus)
  void getOdometry(float &x, float &y, float &theta);

  // reads velocities from internal variable, updated on sync (drive_staus)
  void getVelocities(float &tv, float &rv);

  // issues a velocity command
  void setVelocities(float tv, float rv);

  // reads sonar from internal variable
  void getSonar(bool &is_new, float &range, int sonar_num);

  // retrieves the current epoch (seq of status packet)
  int currentEpoch();

  // does one round of handshake (done by us)

  void sync();

protected:
  OrazioClient *_client = 0;

  // packets we use to send/receive commands to/from the client
  SystemParamPacket system_params;              // read-write
  SystemStatusPacket system_status;             // read
  DifferentialDriveStatusPacket drive_status;   // read
  SonarStatusPacket sonar_status;               // read
  DifferentialDriveControlPacket drive_control; // write
  // activate required subsystems (don't modify it)
  void _toggleDrive();
};
