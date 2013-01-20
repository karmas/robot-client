#include <ctime>
#include <cmath>
#include "OutputHandler.h"
#include "helpers.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


TimeStampedPCL::TimeStampedPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c,
			       long ts)
  : cloud(c), timeStamp(ts) { }



OutputHandler::OutputHandler(ArClientBase *client, PCLViewer *viewer,
    			     int robotColor)
  : myClient(client), myViewer(viewer),
    myRobotCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    myRobotColor(robotColor),
    handleUpdateInfoftr(this, &OutputHandler::handleUpdateInfo),
    handleSensorInfoftr(this, &OutputHandler::handleSensorInfo)
{
  /*
  // Create a handler for handling packet of type 'update'
  myClient->addHandler("update", &handleUpdateInfoftr);
  // Request 'update' packet in cycles of given time in milliseconds.
  myClient->request("update", 1000);

  // Some server packets require packets from client
  ArNetPacket packet;
  packet.strToBuf("laser");
  myClient->addHandler("getSensorCurrent", &handleSensorInfoftr);
  myClient->request("getSensorCurrent", 2000, &packet);
  */

  // set the voxel leaf to 1 cm
  // note: the clouds are storing in mm
  myVoxelLeaf.x = myVoxelLeaf.y = myVoxelLeaf.z = 10.0f;
}

// free up some memory
OutputHandler::~OutputHandler()
{
  /*
  myClient->requestStop("update");
  */
  for (size_t i = 0; i < myRobotInfos.size(); i++)
    delete myRobotInfos[i];
}

// This function displays some positional information on the robot.
void OutputHandler::handleUpdateInfo(ArNetPacket *packet)
{
  const int BUFFER_LENGHT = 256;
  char buffer[BUFFER_LENGHT];

  packet->bufToStr(buffer, BUFFER_LENGHT);
  //cout << "status is " << buffer << endl;
  packet->bufToStr(buffer, BUFFER_LENGHT);
  //cout << "mode is " << buffer << endl;
  packet->bufToByte2();
  int xPosition = (double)packet->bufToByte4();
  int yPosition = (double)packet->bufToByte4();
  //int theta = (double)packet->bufToByte2();

  pcl::PointXYZRGB point;
  point.x = static_cast<float>(xPosition);
  point.y = static_cast<float>(yPosition);
  point.z = 0.0;
  point.rgba = myRobotColor;
  
  myRobotCloud->push_back(point);
  myViewer->addCloud(myRobotCloud, myClient->getHost() + string("robot"));
}

// This function displays some laser readings on the robot.
void OutputHandler::handleSensorInfo(ArNetPacket *packet)
{
  const int BUFFER_LENGHT = 256;
  char buffer[BUFFER_LENGHT];

  int nReadings = packet->bufToByte2();
  cout << myClient->getRobotName() << " sent readings = "
       << nReadings << endl;
  packet->bufToStr(buffer, BUFFER_LENGHT);
  //cout << "sensor name is " << buffer << endl;

  int x, y;
  for (int i = 0; i < 1; i++) {
    x = packet->bufToByte4();
    y = packet->bufToByte4();
    cout << myClient->getRobotName() << " reading " << i + 1 
         << " = " << x << " , " << y << endl;
  }
}



PCLOutputHandler::PCLOutputHandler(ArClientBase *client,
    PCLViewer *viewer, int robotColor,
    int color, int xo, int yo, int to, int rf)
  : OutputHandler(client, viewer, robotColor),
    myLaserCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    handlePCLdataftr(this, &PCLOutputHandler::handlePCLdata),
    myColor(color),
    myXoffset(xo), myYoffset(yo), myThetaOffset(to),
    myRequestFreq(rf)
{
  // add a handler for the data packet
  myClient->addHandler("getPCL", &handlePCLdataftr);
  // then request it every cycle of given milliseconds
  myClient->request("getPCL", myRequestFreq);
  //myClient->requestOnce("getPCL");
}

// Free up memory and stop data request cycle
PCLOutputHandler::~PCLOutputHandler()
{
  myClient->requestStop("getPCL");

  for (size_t i = 0; i < myLaserClouds.size(); i++)
    delete myLaserClouds[i];
}

// function that shows statistical data from the robots
// Since multiple robots will call this function,
// there must be binary locking implemented
void statsDisplay(const char *robotName, ArNetPacket *packet)
{
  static int totalKB = 0;
  static const time_t startTime = time(NULL);

  // getDataLength returns number of 2bytes
  int currKB = packet->getDataLength() * 2 / 1024;
  totalKB += currKB;

  // calculate average transfer rate
  double rate = 0.0;
  time_t timeElapsed = time(NULL) - startTime;
  if (timeElapsed != 0) {
    rate = totalKB/timeElapsed;
  }
  echo("KB/s", rate);
}

// This method is responsible deciphering the packet receivied from the
// server then adding the points to the point cloud. This point cloud
// is displayed in a viewer.
void PCLOutputHandler::handlePCLdata(ArNetPacket *packet)
{
  pcl::PointXYZRGB point;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    tempLaserCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // get time information
  long timeStamp = packet->bufToByte4();

  // get robot location from packet
  // but add offset to translate to global co-ordinates
  point.x = static_cast<float>(packet->bufToDouble()) + myXoffset;
  point.y = static_cast<float>(packet->bufToDouble()) + myYoffset;
  point.z = 0.0;
  point.rgba = myRobotColor;

  // get robot heading
  double th = packet->bufToDouble();

  // store the robot position when the scan is taken
  myRobotInfos.push_back(new RobotInfo(point, timeStamp, th));

  myRobotCloud->push_back(point);
  myViewer->addCloud(myRobotCloud, myClient->getHost() + string("robot"));

  // get number of points
  int nPoints = packet->bufToByte4();

  // extraction of point co-ordinates
  for (int i = 0; i < nPoints; i++) {
    // displacement
    point.x = static_cast<float>(packet->bufToDouble()) + myXoffset;
    point.y = static_cast<float>(packet->bufToDouble()) + myYoffset;
    point.z = static_cast<float>(packet->bufToDouble());
    point.rgba = myColor; 

    setMinMax(point);
    tempLaserCloud->push_back(point);
    // also add point in aggregate cloud which is used by viewer
    myLaserCloud->push_back(point);
  }

  // create time stamped cloud and store it in list
  myLaserClouds.push_back(new TimeStampedPCL(tempLaserCloud, timeStamp));

  myLaserCloud = voxelFilter(myLaserCloud, myVoxelLeaf);

  // Display the laser points using the aggregate cloud not the list
  // because the viewer refreshes each time a cloud is added.
  myViewer->addCloud(myLaserCloud, myClient->getHost() + string("laser"));

  //statsDisplay(myClient->getRobotName(), packet);
  //myClient->logTracking(true);

  //cout << "DENSITY " << calcAvgDensity() << endl;
  //calcAvgDensity();
}

// Writes current PCL cloud to a file in the following format
// [filename]_[month]_[day]_[time]
void PCLOutputHandler::createFile(const char *filename)
{
  const char SEPARATOR = '_';
  const char DATE_SEPARATOR = '-';
  const char TIME_SEPARATOR = ':';
  time_t seconds = time(NULL);
  struct tm *timeInfo = localtime(&seconds);

  stringstream new_name;
  new_name << filename << SEPARATOR
           << timeInfo->tm_mon << DATE_SEPARATOR
           << timeInfo->tm_mday << SEPARATOR
	   << timeInfo->tm_hour << TIME_SEPARATOR
	   << timeInfo->tm_min << TIME_SEPARATOR
	   << timeInfo->tm_sec << ".pcd";

  if (myLaserCloud->width > 0) {
    pcl::io::savePCDFile(new_name.str(), *myLaserCloud);
    echo("NEW PCD FILE", new_name.str());
  }
  else {
    echo("NO CLOUD POINTS. LASER OFF ???");
  }
}

// only used for debugging
void PCLOutputHandler::printClouds()
{
  echo("printing all clouds");
  TimeStampedPCL *curr = NULL;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  for (size_t i = 0; i < myLaserClouds.size() && i < 2; i++) {
    curr = myLaserClouds[i];
    cout << "\tcloud " << i << endl;

    cloud = curr->getCloud();
    for (size_t j = 0; j < 10; j++) {
      cout << "point " << j << ": ";
      cout << (*cloud)[j].x << endl;
    }
  }
}

// set min and max values if possible
void PCLOutputHandler::setMinMax(const pcl::PointXYZRGB &point)
{
  static bool firstTime = true;

  // the first ever point is used to initialize the min and max values
  if (firstTime) {
    firstTime = false;
    myMinVals.x = myMaxVals.x = point.x;
    myMinVals.y = myMaxVals.y = point.y;
    myMinVals.z = myMaxVals.z = point.z;
    return;
  }

  if (point.x < myMinVals.x) myMinVals.x = point.x;
  else if (point.x > myMaxVals.x) myMaxVals.x = point.x;
  if (point.y < myMinVals.y) myMinVals.y = point.y;
  else if (point.y > myMaxVals.y) myMaxVals.y = point.y;
  if (point.z < myMinVals.z) myMinVals.z = point.z;
  else if (point.z > myMaxVals.z) myMaxVals.z = point.z;
}

// Calculate the point density which is the number of points
// in a selected volume. The default units for the laser is mm.
// Hence the values have to be scaled to desired units.
double PCLOutputHandler::calcAvgDensity()
{
  // this is mm to decimeter (10cm);
  const int factor = 100;
  const string units = "dm";
  double l, b, h, v;

  // range of x values
  if ((myMinVals.x >= 0.0 && myMaxVals.x >= 0.0) ||
      (myMinVals.x <= 0.0 && myMaxVals.x <= 0.0))
    l = fabs(fabs(myMaxVals.x) - fabs(myMinVals.x));
  else
    l = fabs(myMaxVals.x) + fabs(myMinVals.x);
  // range of y values
  if ((myMinVals.y >= 0.0 && myMaxVals.y >= 0.0) ||
      (myMinVals.y <= 0.0 && myMaxVals.y <= 0.0))
    b = fabs(fabs(myMaxVals.y) - fabs(myMinVals.y));
  else
    b = fabs(myMaxVals.y) + fabs(myMinVals.y);
  // range of z values
  if ((myMinVals.z >= 0.0 && myMaxVals.z >= 0.0) ||
      (myMinVals.z <= 0.0 && myMaxVals.z <= 0.0))
    h = fabs(fabs(myMaxVals.z) - fabs(myMinVals.z));
  else
    h = fabs(myMaxVals.z) + fabs(myMinVals.z);

  // scale the measurements to desired units
  l = l/factor;
  b = b/factor;
  h = h/factor;

  // calculate density
  v = l * b * h;
  //cout << "\tchosen units " << units << endl;
  //cout << "L " << l << "  ";
  //cout << "B " << b << "  ";
  //cout << "H " << h << "  ";
  //cout << "VOL " << v << "  ";

  calcRegionDensity(myLaserCloud, myMinVals, myMaxVals, units);

  return myLaserCloud->size() / v;
}



PCLViewer::PCLViewer(const std::string& title)
  : myViewer(title)
{
}

void PCLViewer::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    			 const std::string& name)
{
  if (myViewer.wasStopped()) return;

  myViewer.showCloud(cloud, name);
}

void PCLViewer::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if (myViewer.wasStopped()) return;

  myViewer.showCloud(cloud);
}

// Adds cloud in the time stamped cloud.
// Since adding a cloud refreshes the viewer, it is better to use
// addCloud directly on an aggregate cloud then to call this function
// for every new time stamped cloud.
void PCLViewer::addTimeStampedCloud(TimeStampedPCL *tsCloud)
{
  if (myViewer.wasStopped()) return;
  
  // create an string id
  ostringstream os;
  os << tsCloud->getTimeStamp();

  addCloud(tsCloud->getCloud(), os.str());
}


// Return average density in given region of a point cloud.
// MinVal holds the minimum values for the co-ordinates and maxVal
// holds the maximum values. These two points represent the furthest
// points in a cuboid region of space.
// The units parameter is checked against a number of possibilites.
double calcRegionDensity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    			 const MyPoint &minVal, const MyPoint &maxVal,
			 const std::string &units)
{

  cout << "cloud size: " << cloud->size() << endl;

  return 0.0;
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
