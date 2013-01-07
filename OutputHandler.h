#ifndef OUTPUT_HANDLER_H
#define OUTPUT_HANDLER_H

#include "ArNetworking.h"
#include "pcl/io/io.h"
#include "pcl/io/file_io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/visualization/cloud_viewer.h>


// This class is responsible for displaying point clouds on a viewer
class PCLViewer {
public:
  PCLViewer(const std::string& title); 
  ~PCLViewer() {}
  void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      		const std::string& name);

private:
  PCLViewer(const PCLViewer&);
  PCLViewer& operator=(const PCLViewer&);

  pcl::visualization::CloudViewer myViewer;
};



// This class handles output data from server. Currently it supports the
// built in Aria server services: ArServerInfoRobot and ArServerInfoSensor
class OutputHandler {
public:
  OutputHandler(ArClientBase *client, PCLViewer *viewer, int robotColor);
  virtual ~OutputHandler();

  void handleUpdateInfo(ArNetPacket *packet);
  void handleSensorInfo(ArNetPacket *packet);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRobotCloud() {
    return myRobotCloud;
  }
  ArClientBase *getClient() { return myClient; }

protected:
  ArClientBase *myClient;
  PCLViewer *myViewer;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr myRobotCloud;
  int myRobotColor;

private:
  ArFunctor1C<OutputHandler, ArNetPacket *> handleUpdateInfoftr;
  ArFunctor1C<OutputHandler, ArNetPacket *> handleSensorInfoftr;
};




// This class provides functionality to handle PCL data packets from server
// and displaying them in a PCL viewer.
class PCLOutputHandler : public OutputHandler {
public:
  PCLOutputHandler(ArClientBase *client, PCLViewer *viewer, int robotColor,
		   int color, int xo, int yo, int to, int rf = 1000);
  ~PCLOutputHandler();
  void handlePCLdata(ArNetPacket *packet);
  void createFile(const char *filename);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLaserCloud() {
    return myLaserCloud;
  }

private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr myLaserCloud;
  ArFunctor1C<PCLOutputHandler, ArNetPacket *> handlePCLdataftr;
  int myColor;
  int myXoffset;
  int myYoffset;
  int myThetaOffset;
  int myRequestFreq;
};



#endif
