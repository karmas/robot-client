#include <cmath>

#include "helpers.h"
#include "PCLutils.h"
#include "OutputHandler.h"


TimeStampedPCL::TimeStampedPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c,
			       long ts)
  : cloud(c), timeStamp(ts) { }



// Initialize the viewer window
PCLViewer::PCLViewer(const std::string& title,
    		     std::vector<PCLOutputHandler *> &clients)
  : myViewer(title), myClients(clients)
{
#ifdef PCLVISUALIZER
  myViewer.setBackgroundColor(0,0,0);
  myViewer.addCoordinateSystem(170.0);
  myViewer.initCameraParameters();
#endif
}

// Add a cloud or update it if it has already been added before
void PCLViewer::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    			 const std::string& name)
{
  if (myViewer.wasStopped()) return;
#ifdef PCLVISUALIZER
  // give viewer time to process events
  myViewer.spinOnce(100);

  if (!myViewer.updatePointCloud(cloud, name))
    myViewer.addPointCloud(cloud, name);
#else
  myViewer.showCloud(cloud, name);
#endif
}


// Return average density in given region of a point cloud.
// MinVal holds the minimum values for the co-ordinates and maxVal
// holds the maximum values. These two points represent the furthest
// points in a cuboid region of space.
// Also uses the given divison value to convert to required units
double calcRegionDensity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    			 const MyPoint &minVal, const MyPoint &maxVal,
			 int divisor)
{
  // get number of points in the region
  int nPoints = 0;
  for (size_t i = 0; i < cloud->size(); i++) {
    if (minVal.x <= (*cloud)[i].x && maxVal.x >= (*cloud)[i].x &&
        minVal.y <= (*cloud)[i].y && maxVal.y >= (*cloud)[i].y &&
        minVal.z <= (*cloud)[i].z && maxVal.z >= (*cloud)[i].z) {
      nPoints++;
    }
  }

  // get the volume of the region
  double l = 0.0, b = 0.0, h = 0.0;
  // the min and max of a dimension could have the same sign
  if ((minVal.x < 0.0 && maxVal.x < 0.0) ||
      (minVal.x > 0.0 && maxVal.x > 0.0))
    l = fabs(fabs(maxVal.x) - fabs(minVal.x));
  else
    l = fabs(minVal.x) + fabs(maxVal.x);
  if ((minVal.y < 0.0 && maxVal.y < 0.0) ||
      (minVal.y > 0.0 && maxVal.y > 0.0))
    b = fabs(fabs(maxVal.y) - fabs(minVal.y));
  else
    b = fabs(minVal.y) + fabs(maxVal.y);
  if ((minVal.z < 0.0 && maxVal.z < 0.0) ||
      (minVal.z > 0.0 && maxVal.z > 0.0))
    h = fabs(fabs(maxVal.z) - fabs(minVal.z));
  else
    h = fabs(minVal.z) + fabs(maxVal.z);

  // scale the volume to preferred units
  l /= divisor;
  b /= divisor;
  h /= divisor;
  double v = (h > 0.0) ? l*b*h : l*b;

  return nPoints/v;
}

// Perform voxel filter and return filtered cloud
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
voxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
    	    const MyPoint &leafSize)
{
  // this object performs the filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  voxelGrid.setLeafSize(leafSize.x, leafSize.y, leafSize.z);

  // filtered cloud is stored here
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // filter and set to the filtered cloud
  voxelGrid.setInputCloud(source);
  voxelGrid.filter(*filteredCloud);
  return filteredCloud;
}
