#include <ctime>
#include <cmath>

#include "helpers.h"
#include "OutputHandler.h"

// useful constants for the kalman filter
const int stateDims = 2;
const int measurementDims = 2;
const float processNoiseCovValue = 1e-5;
const float measurementNoiseCovValue = 1e-2;


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
	new cv::KalmanFilter(stateDims, measurementDims)),
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
  // state transition model
  setIdentity(kalmanFilter->transitionMatrix);
  setIdentity(kalmanFilter->processNoiseCov, 
      	      cv::Scalar::all(processNoiseCovValue));
  // measurement model
  setIdentity(kalmanFilter->measurementMatrix);
  setIdentity(kalmanFilter->measurementNoiseCov,
      	      cv::Scalar::all(measurementNoiseCovValue));
  //setIdentity(kalmanFilter->errorCovPost);

  // initial state is random
  // set to starting location in PCLOutputHandler constructor
  randn(kalmanFilter->statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

  // method to input individual elements
  //kalmanFilter->transitionMatrix = 
    //*(cv::Mat_<float>(stateDims, stateDims) << 1, 0, 0, 1);
}

// free up some memory
OutputHandler::~OutputHandler()
{
  /*
  myClient->requestStop("update");
  */
  for (size_t i = 0; i < myRobotInfos.size(); i++)
    delete myRobotInfos[i];
  delete kalmanFilter;
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
  // initial state is robot's starting pose
  kalmanFilter->statePost.at<float>(0) = 0 + myXoffset;
  kalmanFilter->statePost.at<float>(1) = 0 + myYoffset;

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

// This method is responsible deciphering the packet receivied from the
// server then adding the points to the point cloud. This point cloud
// is displayed in a viewer.
void PCLOutputHandler::handlePCLdata(ArNetPacket *packet)
{
  long timeStamp = packet->bufToByte4();
  updateRobotLocation(packet, timeStamp);
  updateLaserReadings(packet, timeStamp);

  //myClient->logTracking(true);

  //std::cout << "Density in whole point cloud (points/dm) = ";
  //std::cout << calcRegionDensity(myLaserCloud, myMinVals, myMaxVals, 
  //    			    myDensityDivisor);
  //std::cout << std::endl;
}

// Extract robot location from packet and update the clouds holding
// robot locations.
void PCLOutputHandler::updateRobotLocation(ArNetPacket *packet,
    long timeStamp)
{
  pcl::PointXYZRGB point;
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

  // also display the robot position
  myRobotCloud->push_back(point);
  myViewer->addCloud(myRobotCloud, 
      myClient->getHost() + std::string("robot"));

  filterRobotLocation(point);
}

// kalman filtering of robot position
void PCLOutputHandler::filterRobotLocation(pcl::PointXYZRGB &measured)
{
  // get previous state
  cv::Mat state(2, 1, CV_32F);
  state = kalmanFilter->statePost;

  kalmanFilter->predict();	// perform prediction

  // fill measurement matrix with values from odometer
  cv::Mat measurement(2, 1, CV_32F);
  measurement.at<float>(0) = measured.x;
  measurement.at<float>(1) = measured.y;
  // generate measurement according to model
  //measurement += kalmanFilter->measurementMatrix * state;

  // adjust kalman filter state with measurement
  kalmanFilter->correct(measurement);

  // random noise for the process
  cv::Mat processNoise(2, 1, CV_32F);
  // randn(output array of random numbers,
  //       mean value of generated random numbers,
  //       stddev of random numbers)
  randn(processNoise, cv::Scalar(0),
        cv::Scalar::all(sqrt(kalmanFilter->processNoiseCov.at<float>(0,0))));

  // get updated state from model
  state = kalmanFilter->transitionMatrix * state + processNoise;
  // retreive state values and give it white color for display
  pcl::PointXYZRGB pointFiltered;
  pointFiltered.x = state.at<float>(0);
  pointFiltered.y = state.at<float>(1);
  pointFiltered.z = 0;
  pointFiltered.rgba = rgba(255,255,255);

//  std::cout << " x = " << measured.x << " || " << pointFiltered.x << " , "
//            << " y = " << measured.y << " || " << pointFiltered.y
//	    << std::endl;

  // remember the filtered positions and display
  myRobotCloudFiltered->push_back(pointFiltered);
  myViewer->addCloud(myRobotCloudFiltered,
      		     myClient->getHost() + std::string("robotFiltered"));
}

// Extract laser readings from packet and update the clouds holding
// laser readings.
void PCLOutputHandler::updateLaserReadings(ArNetPacket *packet, 
    long timeStamp)
{
  pcl::PointXYZRGB point;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    tempLaserCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // get number of laser readings
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

  // downsample the current laser cloud and store it
  tempLaserCloud = voxelFilter(tempLaserCloud, myVoxelLeaf);
  myLaserClouds.push_back(new TimeStampedPCL(tempLaserCloud, timeStamp));

  // downsample the aggregate laser cloud
  myLaserCloud = voxelFilter(myLaserCloud, myVoxelLeaf);

  // Display the laser points using the aggregate cloud not the list
  // because the viewer refreshes each time a cloud is added.
  myViewer->addCloud(myLaserCloud,
      		     myClient->getHost() + std::string("laser"));
}

// set min and max values if possible which can be later used
// to get volume of the region scanned
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
