#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <vector>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

// forward declarations
class PCLOutputHandler;

// just to aggregate data
struct MyPoint {
  float x, y, z;
};

// group robot position, heading and timestamp
struct RobotInfo {
  RobotInfo(const pcl::PointXYZRGB &pt, long ts, double h)
    : point(pt), timeStamp(ts), th(h) { }
  pcl::PointXYZRGB point;
  long timeStamp;
  double th;	// heading in degrees
};

// a time stamped point cloud
class TimeStampedPCL {
public:
  TimeStampedPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c, long ts);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud() { return cloud; }
  long getTimeStamp() { return timeStamp; }

private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  long timeStamp;

  // make copying illegal
  TimeStampedPCL(const TimeStampedPCL &) {}
  TimeStampedPCL &operator=(const TimeStampedPCL &) { return *this; }
};



// This class is responsible for displaying point clouds on a viewer
class PCLViewer {
public:
  PCLViewer(const std::string& title,
      	    std::vector<PCLOutputHandler *> &clients); 
  ~PCLViewer() {}
  void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      		const std::string& name);

private:
  PCLViewer(const PCLViewer&);
  PCLViewer& operator=(const PCLViewer&);

#ifdef PCLVISUALIZER
  pcl::visualization::PCLVisualizer myViewer;
#else
  pcl::visualization::CloudViewer myViewer;
#endif
  std::vector<PCLOutputHandler *> &myClients;
};


// Some helpful functions

double calcRegionDensity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    			 const MyPoint &minVal, const MyPoint &maxVal,
			 int divisor);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
voxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
    	    const MyPoint &leafSize);


#endif
