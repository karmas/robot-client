#ifndef SENSOR_DATA_HANDLER_H
#define SENSOR_DATA_HANDLER_H

#include <vector>

#include "ArNetworking.h"

#include "pcl/io/io.h"
#include "pcl/io/file_io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <cv.h>

#include "PCLutils.h"
#include "ConfigFileReader.h"

// Abstract base class which provides the interface for class that
// handle sensor data from the robot
class SensorDataHandler {
public:
  virtual void request() = 0;
  virtual void writeTo(const std::string &outDir) = 0;
  MyCloud::Ptr getDisplayCloud();
  void writeDisplayCloud(const std::string &outDir);

protected:
  SensorDataHandler(ArClientBase *client, const char *dataName,
      		    int requestFreq);
  virtual ~SensorDataHandler();
  virtual void handle(ArNetPacket *packet) = 0;

  ArClientBase *myClient;
  const char *myDataName;
  const int myRequestFreq;
  MyCloud::Ptr myDisplayCloud;
};


// handle laser data
class SensorDataLaserHandler : public SensorDataHandler {
public:
  SensorDataLaserHandler(ArClientBase *client, const HostInfo &hostInfo);
  ~SensorDataLaserHandler();
  virtual void request();
  virtual void handle(ArNetPacket *packet);
  virtual void writeTo(const std::string &outDir);

  static const double pi;
  static const double toRadian;

private:
  void updateRobotLocation(ArNetPacket *packet, long timeStamp);
  void updateLaserReadings(ArNetPacket *packet, long timeStamp);
  void filterRobotLocation(MyPoint &measured);

  ArFunctor1C<SensorDataLaserHandler, ArNetPacket *> myHandleFtr;
  std::vector<RobotInfo *> myRobotInfos;
  MyCloud::Ptr myRobotCloud;
  const int myRobotColor;
  std::vector<TimeStampedPCL *> myLaserClouds;
  const int myLaserColor;
  const TransformInfo myTransformInfo;
  const double myCosTheta;
  const double mySinTheta;
  MyPoint myVoxelLeaf;
  MyCloud::Ptr myRobotCloudFiltered;
  cv::KalmanFilter *myKalmanFilter;
};

// handle stereo camera data
class SensorDataStereoCamHandler : public SensorDataHandler {
public:
  SensorDataStereoCamHandler(ArClientBase *client, const HostInfo &hostInfo);
  ~SensorDataStereoCamHandler();
  virtual void request();
  virtual void handle(ArNetPacket *packet);
  MyCloud::Ptr displayCloud();
  void writeTo(const std::string &outDir);

private:
  ArFunctor1C<SensorDataStereoCamHandler, ArNetPacket *> myHandleFtr;
};



// helper functions for sensor data handlers
void createSensorDataHandlers(
    std::vector<ArClientBase *> &clients,
    std::vector<SensorDataHandler *> &sensorDataHandlers,
    std::vector<HostInfo> &hostsInfo);
void writeSensorDataToDisk(
    std::vector<SensorDataHandler *> &sensorDataHandlers);



#endif
