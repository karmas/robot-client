#include <ctime>
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
}

OutputHandler::~OutputHandler()
{
  /*
  myClient->requestStop("update");
  */
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

PCLOutputHandler::~PCLOutputHandler()
{
  myClient->requestStop("getPCL");
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
  TimeStampedPCL *tempLaserTS = NULL;

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

  //myRobotCloud->push_back(point);
  //myViewer->addCloud(myRobotCloud, myClient->getHost() + string("robot"));

  // get number of points
  int nPoints = packet->bufToByte4();

  // extraction of point co-ordinates
  for (int i = 0; i < nPoints; i++) {
    // displacement
    point.x = static_cast<float>(packet->bufToDouble()) + myXoffset;
    point.y = static_cast<float>(packet->bufToDouble()) + myYoffset;
    point.z = static_cast<float>(packet->bufToDouble());
    point.rgba = myColor; 

    tempLaserCloud->push_back(point);
    //myLaserCloud->push_back(point);
  }

  // create time stamped aggregate class and store the cloud
  // then store in the list
  tempLaserTS = new TimeStampedPCL(tempLaserCloud, timeStamp);
  myLaserClouds.push_back(tempLaserTS);

  // display it as well
  //myViewer->addCloud(tempLaserCloud, myClient->getHost() + string("laser"));
  ostringstream os;
  os << timeStamp;
  myViewer->addCloud(tempLaserCloud, os.str());

  //statsDisplay(myClient->getRobotName(), packet);
  //myClient->logTracking(true);
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
