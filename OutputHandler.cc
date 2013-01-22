#include <ctime>
#include <cmath>

#include "helpers.h"
#include "OutputHandler.h"

// useful constants for the kalman filter
const int stateDimensions = 2;
const int measurementDimensions = 2;

TimeStampedPCL::TimeStampedPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c,
			       long ts)
  : cloud(c), timeStamp(ts) { }


// Density is set to points per cubic/square dm
const int OutputHandler::myDensityDivisor = 100;

// Kalman filter is applied to estimate the robot position which consists
// of x and y co-ordinate values.
// The measurement is the odometer reading from the robot itself which
// consists of x and y co-ordinate values.
// Currently control information is not incorporated. The directional
// velocities could be used.
OutputHandler::OutputHandler(ArClientBase *client, PCLViewer *viewer,
    			     int robotColor)
  : myClient(client), myViewer(viewer),
    myRobotCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    myRobotColor(robotColor),
    myRobotCloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>),
    kalmanFilter(
	new cv::KalmanFilter(stateDimensions, measurementDimensions)),
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

  ////////////////////////////////////////////////
  // initialize the model for the kalman filter//
  // ////////////////////////////////////////////
  // set the transition matrix to [ 1 0 ]
  // 				  [ 0 1 ]
  kalmanFilter->transitionMatrix = *(cv::Mat_<float>(2, 2) << 1, 0, 0, 1);
  //setIdentity(kalmanFilter->measurementMatrix);
  setIdentity(kalmanFilter->processNoiseCov, cv::Scalar::all(1e-5));
  setIdentity(kalmanFilter->measurementNoiseCov, cv::Scalar::all(1e-2));
  //setIdentity(kalmanFilter->errorCovPost, cv::Scalar::all(1));
  // initial state is random
  randn(kalmanFilter->statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
}

// free up some memory
OutputHandler::~OutputHandler()
{
  /*
  myClient->requestStop("update");
  */
  for (size_t i = 0; i < myRobotInfos.size(); i++)
    delete myRobotInfos[i];
  ////delete kalmanFilter;
}

// This function displays some positional information on the robot.
void OutputHandler::handleUpdateInfo(ArNetPacket *packet)
{
  const int BUFFER_LENGHT = 256;
  char buffer[BUFFER_LENGHT];

  packet->bufToStr(buffer, BUFFER_LENGHT);
  //std::cout << "status is " << buffer << std::endl;
  packet->bufToStr(buffer, BUFFER_LENGHT);
  //std::cout << "mode is " << buffer << std::endl;
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
  myViewer->addCloud(myRobotCloud, myClient->getHost() + std::string("robot"));
}

// This function displays some laser readings on the robot.
void OutputHandler::handleSensorInfo(ArNetPacket *packet)
{
  const int BUFFER_LENGHT = 256;
  char buffer[BUFFER_LENGHT];

  int nReadings = packet->bufToByte2();
  std::cout << myClient->getRobotName() << " sent readings = "
       << nReadings << std::endl;
  packet->bufToStr(buffer, BUFFER_LENGHT);
  //std::cout << "sensor name is " << buffer << std::endl;

  int x, y;
  for (int i = 0; i < 1; i++) {
    x = packet->bufToByte4();
    y = packet->bufToByte4();
    std::cout << myClient->getRobotName() << " reading " << i + 1 
         << " = " << x << " , " << y << std::endl;
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
  myViewer->addCloud(myRobotCloud, myClient->getHost() + std::string("robot"));

  // kalman filtering of robot position
  // get previous state
  cv::Mat state(2, 1, CV_32F);
  state = kalmanFilter->statePost;
  kalmanFilter->predict();	// perform prediction
  // fill measurement matrix with values from odometer
  cv::Mat measurement(2, 1, CV_32F);
  measurement.at<float>(0) = point.x;
  measurement.at<float>(1) = point.y;
  // adjust kalman filter state with measurement
  kalmanFilter->correct(measurement);
  // random noise for the process
  cv::Mat processNoise(2, 1, CV_32F);
  // randn(output array of random numbers,
  //       mean value of generated random numbers,
  //       stddev of random numbers)
  randn(processNoise, cv::Scalar(0),
        cv::Scalar::all(sqrt(kalmanFilter->processNoiseCov.at<float>(0,0))));
  // get current state from model
  state = kalmanFilter->transitionMatrix*state + processNoise;
  // retreive state values and give it white color for display
  pcl::PointXYZRGB pointFiltered;
  pointFiltered.x = state.at<float>(0);
  pointFiltered.y = state.at<float>(1);
  pointFiltered.z = 0;
  pointFiltered.rgba = rgba(0,0,255);

  std::cout << " x = " << point.x << " || " << pointFiltered.x << " , "
            << " y = " << point.y << " || " << pointFiltered.y
	    << std::endl;

  // remember the filtered positions and display
  myRobotCloudFiltered->push_back(pointFiltered);
  myViewer->addCloud(myRobotCloudFiltered,
      		     myClient->getHost() + std::string("robotFiltered"));

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
  myViewer->addCloud(myLaserCloud,
      		     myClient->getHost() + std::string("laser"));

  //statsDisplay(myClient->getRobotName(), packet);
  //myClient->logTracking(true);

  //std::cout << "Density in whole point cloud (points/dm) = ";
  //std::cout << calcRegionDensity(myLaserCloud, myMinVals, myMaxVals, 
  //    			    myDensityDivisor);
  //std::cout << std::endl;
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

  std::stringstream new_name;
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
    std::cout << "\tcloud " << i << std::endl;

    cloud = curr->getCloud();
    for (size_t j = 0; j < 10; j++) {
      std::cout << "point " << j << ": ";
      std::cout << (*cloud)[j].x << std::endl;
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
  std::ostringstream os;
  os << tsCloud->getTimeStamp();

  addCloud(tsCloud->getCloud(), os.str());
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
