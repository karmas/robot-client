#include <cmath>

#include "helpers.h"
#include "PCLutils.h"
#include "OutputHandler.h"


TimeStampedPCL::TimeStampedPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c,
			       long ts)
  : cloud(c), timeStamp(ts) { }


// Handle the key presses in the cloud viewer window
// This is causing the window to hang up.
void handleKeyboadEvents(const pcl::visualization::KeyboardEvent &keyEvent,
    			 void *arg)
{
  // get the viewer
  PCLViewer *viewer = static_cast<PCLViewer *>(arg);

  if (keyEvent.getKeySym() == "t" && keyEvent.keyDown())
    viewer->startTimeDemo();

  return;
}


// Initialize the viewer window
PCLViewer::PCLViewer(const std::string& title,
    		     std::vector<PCLOutputHandler *> &clients)
  : myViewer(title), myClients(clients), myDemoState(false)
{
  myViewer.setBackgroundColor(0,0,0);
  myViewer.addCoordinateSystem(170.0);
  myViewer.initCameraParameters();
  //myViewer.registerKeyboardCallback(handleKeyboadEvents, (void*)this);
}

// Add a cloud or update it if it has already been added before
void PCLViewer::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    			 const std::string& name)
{
  if (myViewer.wasStopped()) return;
  // give viewer time to process events
  myViewer.spinOnce(100);

  if (!myViewer.updatePointCloud(cloud, name))
    myViewer.addPointCloud(cloud, name);

  //myViewer.setPointCloudRenderingProperties(
   //   pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
}

// Adds cloud in the time stamped cloud.
// Since adding a cloud refreshes the viewer, it is better to use
// addCloud directly on an aggregate cloud then to call this function
// for every new time stamped cloud.
void PCLViewer::addTimeStampedCloud(TimeStampedPCL *tsCloud)
{
  if (myViewer.wasStopped()) return;
  
  // create an string id
  std::ostringstream os;
  os << tsCloud->getTimeStamp();

  addCloud(tsCloud->getCloud(), os.str());
}

// Start demo for time stamped point clouds and suspend normal view.
// It iterates through each point cloud.
// Shows the point cloud.
// Pauses.
// Then Repeats.
void PCLViewer::startTimeDemo()
{
  myDemoState = true;
  echo("TIME STAMP DEMO MODE");

  // clear the display window
  myViewer.removeAllPointClouds();
  myViewer.spinOnce(200);

  size_t nClients = myClients.size();

  // This array of pointers will be used to refer to vector of time stamped
  // point clouds stored in each client.
  std::vector<TimeStampedPCL *> *tsClouds[nClients];

  // Go through each client.
  for (size_t i = 0; i < nClients; i++) {
    // Stop the data transer.
    myClients[i]->getClient()->requestStop("getPCL");
    // Remember the collection of time stamped point clouds
    tsClouds[i] = myClients[i]->getLaserClouds();
  }

  // Number of time steps should be same for each client's collection so
  // just use the first client's info
  size_t timeSteps = tsClouds[0]->size();

  // Now display point clouds from the collection in sequence with time
  // gap.
  for (size_t ts = 0; ts < timeSteps; ts++) {
    for (size_t j = 0; j < nClients; j++) {
      // for each collection, display the point cloud indexed by cloudIndex
      addCloud((*tsClouds[j])[ts]->getCloud(), "laser");
    }
    ArUtil::sleep(500);
    // clear the display window
    myViewer.removeAllPointClouds();
    myViewer.spinOnce(100);
  }
}

// Stop demo for time stamped point clouds and resume normal view
void PCLViewer::stopTimeDemo()
{
  myDemoState = false;
  echo("NORMAL MODE");

  // clear the display window
  myViewer.removeAllPointClouds();
  myViewer.spinOnce(200);

  // continue data transfer
  for (size_t i = 0; i < myClients.size(); i++)
    myClients[i]->getClient()->request("getPCL", 1000);
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
