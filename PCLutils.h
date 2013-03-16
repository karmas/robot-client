#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#include <vector>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

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
class TimeStampedPCL {
public:
  TimeStampedPCL(MyCloud::Ptr c, long ts);
  MyCloud::Ptr getCloud() { return cloud; }
  long getTimeStamp() { return timeStamp; }

private:
  MyCloud::Ptr cloud;
  long timeStamp;

  // make copying illegal
  TimeStampedPCL(const TimeStampedPCL &) {}
  TimeStampedPCL &operator=(const TimeStampedPCL &) { return *this; }
};



// Some helpful functions

double calcRegionDensity(MyCloud::Ptr cloud,
    			 const MyPoint &minVal, const MyPoint &maxVal,
			 int divisor);
MyCloud::Ptr voxelFilter(MyCloud::Ptr source, const MyPoint &leafSize);


#endif
