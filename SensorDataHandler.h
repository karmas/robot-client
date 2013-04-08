#ifndef SENSOR_DATA_HANDLER_H
#define SENSOR_DATA_HANDLER_H

#include <vector>

#include "ArNetworking.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "ConfigFileReader.h"


// forward declaration
namespace cv {
  class KalmanFilter;
}

typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointCloud<MyPoint> MyCloud;

// group robot position, heading and timestamp
struct RobotInfo {
  RobotInfo(const MyPoint &pt, long ts, double h)
    : point(pt), timeStamp(ts), th(h) { }
  MyPoint point;
  long timeStamp;
  double th;	// heading in degrees
};

// a time stamped point cloud
class TSCloud {
public:
  TSCloud(MyCloud::Ptr c, long ts)
    : cloud(c), timeStamp(ts) { }
  MyCloud::Ptr getCloud() { return cloud; }
  long getTimeStamp() { return timeStamp; }

private:
  MyCloud::Ptr cloud;
  long timeStamp;

  // make copying illegal
  TSCloud(const TSCloud &) {}
  TSCloud &operator=(const TSCloud &) { return *this; }
};




// Abstract base class which provides the interface for class that
// handle sensor data from the robot
class SensorDataHandler {
public:
  virtual void request() = 0;
  virtual void writeTo(const std::string &outDir);
  MyCloud::Ptr getDisplayCloud();
  MyPoint transformPoint(const ArPose &fromFrame, const MyPoint &point);

  static const double pi;
  static const double toRadian;

protected:
  SensorDataHandler(ArClientBase *client, const char *dataName,
      		    int requestFreq, int myRobotColor);
  virtual ~SensorDataHandler();
  virtual void handle(ArNetPacket *packet) = 0;

  ArClientBase *myClient;
  const char *myDataName;
  const int myRequestFreq;
  MyCloud::Ptr myDisplayCloud;
  MyPoint myVoxelLeaf;
  std::vector<RobotInfo *> myRobotInfos;
  MyCloud::Ptr myRobotCloud;
  const int myRobotColor;
  std::vector<TSCloud *> myTSClouds;
};


// handle laser data
class SensorDataLaserHandler : public SensorDataHandler {
public:
  SensorDataLaserHandler(ArClientBase *client, const HostInfo &hostInfo);
  ~SensorDataLaserHandler();
  virtual void request();
  virtual void handle(ArNetPacket *packet);

private:
  void updateRobotLocation(ArNetPacket *packet, long timeStamp);
  void updateLaserReadings(ArNetPacket *packet, long timeStamp);
  void filterRobotLocation(MyPoint &measured);

  ArFunctor1C<SensorDataLaserHandler, ArNetPacket *> myHandleFtr;
  const int myLaserColor;
  const TransformInfo myTransformInfo;
  const double myCosTheta;
  const double mySinTheta;
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
  virtual void handle2(ArNetPacket *packet);

private:
  ArFunctor1C<SensorDataStereoCamHandler, ArNetPacket *> myHandleFtr;
  const int myStatFilterK;
  const char *myDataName2;
  ArFunctor1C<SensorDataStereoCamHandler, ArNetPacket *> myHandleFtr2;
};



// helper functions for sensor data handlers
void createSensorDataHandlers(
    std::vector<ArClientBase *> &clients,
    std::vector<SensorDataHandler *> &sensorDataHandlers,
    std::vector<HostInfo> &hostsInfo);
void writeSensorDataToDisk(
    std::vector<SensorDataHandler *> &sensorDataHandlers);
double calcRegionDensity(MyCloud::Ptr cloud,
    			 const MyPoint &minVal, const MyPoint &maxVal,
			 int divisor);
MyCloud::Ptr voxelFilter(MyCloud::Ptr source, const MyPoint &leafSize);
MyCloud::Ptr statsFilter(MyCloud::Ptr source, const int k);


#endif
