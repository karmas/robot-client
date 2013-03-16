#include <ctime>
#include <cmath>

#include "helpers.h"
#include "SensorDataHandler.h"

//#define KALMAN_FILTER

// useful constants for the kalman filter
const int stateDims = 2;
const int measurementDims = 2;
const float processNoiseCovValue = 1e-1;
const float measurementNoiseCovValue = 1e-2;


SensorDataHandler::SensorDataHandler(ArClientBase *client, 
    const char *dataName, int requestFreq)
  : myClient(client), myDataName(strdup(dataName)),
    myRequestFreq(requestFreq), myDisplayCloud(new MyCloud)
{
}

// free up resources
SensorDataHandler::~SensorDataHandler()
{
  delete myDataName;
}


const double SensorDataLaserHandler::pi = 3.14159165f;
const double SensorDataLaserHandler::toRadian = pi/180;

// Attach packet handler.
//
// Kalman filter is applied to estimate the robot position which consists
// of x and y co-ordinate values.
// The measurement is the odometer reading from the robot itself which
// consists of x and y co-ordinate values.
// Currently control information is not incorporated. The directional
// velocities could be used.
SensorDataLaserHandler::SensorDataLaserHandler(ArClientBase *client,
    const HostInfo &hostInfo)
  : SensorDataHandler(client, "getSensorDataLaser", hostInfo.requestFreq),
    myHandleFtr(this, &SensorDataLaserHandler::handle),
    myRobotCloud(new MyCloud),
    myRobotColor(hostInfo.locationColor),
    myLaserColor(hostInfo.laserColor),
    myTransformInfo(hostInfo.transformInfo.xOffset,
		  hostInfo.transformInfo.yOffset,
		  hostInfo.transformInfo.thetaOffset),
    myCosTheta(cos(myTransformInfo.thetaOffset*toRadian)),
    mySinTheta(sin(myTransformInfo.thetaOffset*toRadian)),
    myRobotCloudFiltered(new MyCloud),
    myKalmanFilter(
	new cv::KalmanFilter(stateDims, measurementDims))
{
  myClient->addHandler(myDataName, &myHandleFtr);

  // set the voxel leaf to 1 cm
  // note: the clouds are storing in mm
  myVoxelLeaf.x = myVoxelLeaf.y = myVoxelLeaf.z = 10.0f;

  ////////////////////////////////////////////////
  // initialize the model for the kalman filter//
  // ////////////////////////////////////////////

  // set the transition matrix to [ 1 0 ]
  // 				  [ 0 1 ]
  // state transition model
  setIdentity(myKalmanFilter->transitionMatrix);
  setIdentity(myKalmanFilter->processNoiseCov, 
      	      cv::Scalar::all(processNoiseCovValue));
  // measurement model
  setIdentity(myKalmanFilter->measurementMatrix);
  setIdentity(myKalmanFilter->measurementNoiseCov,
      	      cv::Scalar::all(measurementNoiseCovValue));
  setIdentity(myKalmanFilter->errorCovPost);

  // initial state is robot's starting pose
  myKalmanFilter->statePost.at<float>(0) = 0 + myTransformInfo.xOffset;
  myKalmanFilter->statePost.at<float>(1) = 0 + myTransformInfo.yOffset;
}

// stop the data requests
SensorDataLaserHandler::~SensorDataLaserHandler()
{
  myClient->requestStop(myDataName);

  for (size_t i = 0; i < myLaserClouds.size(); i++)
    delete myLaserClouds[i];
}

// Decode the data packet received
void SensorDataLaserHandler::handle(ArNetPacket *packet)
{
  long timeStamp = getElapsedTime();
  updateRobotLocation(packet, timeStamp);
  updateLaserReadings(packet, timeStamp);
}

// Start requesting data packets 
void SensorDataLaserHandler::request()
{
  myClient->request(myDataName, myRequestFreq);
}

// Get the full cloud which will be displayed
MyCloud::Ptr SensorDataLaserHandler::displayCloud()
{
  return myDisplayCloud;
}

// Extract robot location from packet and update the clouds holding
// robot locations.
void SensorDataLaserHandler::updateRobotLocation(ArNetPacket *packet,
    long timeStamp)
{
  MyPoint origPoint;
  MyPoint point;

  // get robot location from packet
  // reference frame is individual robot's starting point
  origPoint.x = static_cast<float>(packet->bufToDouble());
  origPoint.y = static_cast<float>(packet->bufToDouble());

  // rotate to global reference frame
  point.x = origPoint.x * myCosTheta + origPoint.y * mySinTheta;
  point.y = origPoint.y * myCosTheta - origPoint.x * mySinTheta;

  // translate to global reference frame
  point.x += myTransformInfo.xOffset;
  point.y += myTransformInfo.yOffset;
  point.z = 0.0;
  point.rgba = myRobotColor; 

  // get robot heading
  double th = packet->bufToDouble();
  // store the robot position when the scan is taken
  myRobotInfos.push_back(new RobotInfo(point, timeStamp, th));

  // also add to cloud which will displayed
  myDisplayCloud->push_back(point);
  // add to cloud for robot positions so that it can be written
  // as single file
  myRobotCloud->push_back(point);

#ifdef KALMAN_FILTER
  // kalman filter of robot position
  filterRobotLocation(point);
#endif
}

// Extract laser readings from packet and update the clouds holding
// laser readings.
void SensorDataLaserHandler::updateLaserReadings(ArNetPacket *packet, 
    long timeStamp)
{
  MyPoint origPoint;
  MyPoint point;
  MyCloud::Ptr tempLaserCloud(new MyCloud);

  // get number of laser readings
  int nPoints = packet->bufToByte4();

  // extraction of point co-ordinates
  for (int i = 0; i < nPoints; i++) {
    // reference frame is individual robot's starting point
    origPoint.x = static_cast<float>(packet->bufToDouble());
    origPoint.y = static_cast<float>(packet->bufToDouble());
    origPoint.z = static_cast<float>(packet->bufToDouble());

    // rotate to global reference frame
    point.x = origPoint.x * myCosTheta + origPoint.y * mySinTheta;
    point.y = origPoint.y * myCosTheta - origPoint.x * mySinTheta;

    // translate to global reference frame
    point.x += myTransformInfo.xOffset;
    point.y += myTransformInfo.yOffset;
    point.z = origPoint.z;
    point.rgba = myLaserColor; 

    tempLaserCloud->push_back(point);
    // also add point in aggregate cloud which is used by viewer
    myDisplayCloud->push_back(point);
  }

  // downsample the current laser cloud and store it
  tempLaserCloud = voxelFilter(tempLaserCloud, myVoxelLeaf);
  myLaserClouds.push_back(new TimeStampedPCL(tempLaserCloud, timeStamp));
  // downsample the aggregate laser cloud
  myDisplayCloud = voxelFilter(myDisplayCloud, myVoxelLeaf);
}

// kalman filtering of robot position
void SensorDataLaserHandler::filterRobotLocation(MyPoint &measured)
{
  myKalmanFilter->predict();	// perform prediction

  // fill measurement matrix with values from odometer
  cv::Mat measurement(2, 1, CV_32F);
  measurement.at<float>(0) = measured.x;
  measurement.at<float>(1) = measured.y;

  // adjust kalman filter state with measurement
  cv::Mat state(2, 1, CV_32F);
  state = myKalmanFilter->correct(measurement);

  // retreive state values and give it white color for display
  MyPoint pointFiltered;
  pointFiltered.x = state.at<float>(0);
  pointFiltered.y = state.at<float>(1);
  pointFiltered.z = 0;
  pointFiltered.rgba = rgba(255,255,255);

  // remember the filtered positions and display
  myRobotCloudFiltered->push_back(pointFiltered);
  myDisplayCloud->push_back(pointFiltered);
}

// Writes the single point cloud consisting of robot locations to
// given directory. Writes all the laser point clouds to the given
// directory.
void SensorDataLaserHandler::writeTo(const std::string &outDir)
{
  std::string subDir("");
  std::string filePath = "";
  std::string extension = ".pcd";
  MyCloud::Ptr cloud;

  // create subdirectory for this client's data
  subDir = outDir + "/" + myClient->getRobotName();
  if (!genDir(subDir)) return;

  // robot position cloud filename
  filePath = subDir + "/" + "path" + extension;
  pcl::io::savePCDFile(filePath, *myRobotCloud);

  // write the laser cloud files using the timestamp as name
  for (size_t i = 0; i < myLaserClouds.size(); i++) {
    cloud = myLaserClouds[i]->getCloud();
    std::ostringstream os;
    os << myLaserClouds[i]->getTimeStamp();
    filePath = subDir + "/" + os.str() + extension;
    pcl::io::savePCDFile(filePath, *cloud);
  }
}



// Create sensor data handler objects
void createSensorDataHandlers(
    std::vector<ArClientBase *> &clients,
    std::vector<SensorDataHandler *> &sensorDataHandlers,
    std::vector<HostInfo> &hostsInfo)
{
  SensorDataHandler *sensorDataHandler = NULL;

  for (unsigned int i = 0; i < clients.size(); i++) {
    sensorDataHandler = 
      new SensorDataLaserHandler(clients[i], hostsInfo[i]);
    sensorDataHandlers.push_back(sensorDataHandler);
  }
}

// It creates a new ouput folder where all the point clouds will be stored.
// The output folder has name that is based on a chosen prefix which is
// held in the string variable outDirPrefix and the current time.
void writeSensorDataToDisk(
    std::vector<SensorDataHandler *> &sensorDataHandlers)
{
  const std::string outDirPrefix = "clouds";

  // generate a new output directory based on current time
  std::string outDir = outDirPrefix + genTimeStr();
  // error message already spawned by genOutDir
  if (!genDir(outDir)) return;

  for (size_t i = 0; i < sensorDataHandlers.size(); i++)
    sensorDataHandlers[i]->writeTo(outDir);

  std::cout << "Wrote sensor data to: " << outDir << std::endl;
}
