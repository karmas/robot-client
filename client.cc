/* This client program is supposed to provide a means of controlling
 * a robot which has the accompanying server program running on it.
 * The client will allow you to stop or move the robot and even let the
 * robot move automatically avoiding obstacles. Furthermore a point cloud
 * of the robot's reading is displayed in a separate PCL window.
 *
 * Provide a textfile with an IP address on each line corresponding to
 * a robot server. The accompanying server program must be running on
 * those hosts. This program will create clients to connect to each server.
 * The first line on the text file must be:
 * servers 1.0
 *
 * Default controls for the first two clients.
 *                   client     clone
 *
 * (Let the robot wander automatically avoiding obstacles)
 * auto mode          'a'        'q'
 *
 * (Bring the robot to a stop)
 * stop mode          's'        'w'
 *
 * (Move the robot manually in drive mode)
 * drive mode         'd'        'r'
 * go up           up arrow      'i'
 * go down         down arrow    'k'
 * go left         left arrow    'j'
 * go right        right arrow   'l'
 *
 * Press 'f' to create a pcd file of the current point cloud
 */

#include "Aria.h"
#include "ArNetworking.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>
#include "pcl/io/io.h"
#include "pcl/io/file_io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <Eigen/Core>
#include "pcl/point_cloud.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/passthrough.h"

#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"
#include "pcl/registration/ia_ransac.h"

//for statrm -- this conflicts with flann in openCV < 2.2; just upgrade opencv
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"

#include <pcl/visualization/cloud_viewer.h>

#include <cmath>
#include "helpers.h"
#include "ConfigFileReader.h"
#include "MoveRobot.h"
#include "OutputHandler.h"
using namespace std;


// some useful constants
const string outputFolder = "output/";

// shuts down aria 
void escapePressed()
{ 
  Aria::shutdown(); 
}

// used to create a name of form botxxx where xxx is the last 3 digits
// of the IP address
const char *createRobotName(const char *IP)
{
  string ip = IP;
  int suffixStart = ip.rfind(".") + 1;
  string suffix = ip.substr(suffixStart);
  string name = "bot" + suffix;
  return name.c_str();
}

// Connects to each IP address in hostsIP.
// The client objects are stored in clients.
void connectHosts(vector<ArClientBase *> &clients,
                  const vector<HostInfo> &hostsInfo)
{
  ArClientBase *client = NULL;

  for (unsigned int i = 0; i < hostsInfo.size(); i++) {
    client = new ArClientBase;
    client->setRobotName(createRobotName(hostsInfo[i].ip));
    if (!client->blockingConnect(hostsInfo[i].ip, 7272)) {
      echo("unable to connect to", client->getRobotName());
      Aria::shutdown();
      exit(1);
    }
    else {
      echo("connected to", client->getRobotName());
      clients.push_back(client);
    }
  }
}


// Attach keyboard handling to the clients
void createMovementControls(vector<ArClientBase *> &clients, 
                            ArKeyHandler &keyHandler,
                            vector<MoveRobot *> &moveClients)
{
  int ClientKeys[] = { ArKeyHandler::UP, ArKeyHandler::DOWN,
		       ArKeyHandler::LEFT, ArKeyHandler::RIGHT,
		       'a', 'd', 's' };
  int CloneKeys[] = { 'i', 'k', 'j', 'l', 'q', 'e', 'w' };

  MoveRobot *moveHandler = NULL;

  for (unsigned int i = 0; i < clients.size(); i++) {
    if (i == 0)
      moveHandler = new MoveRobot(clients[i], &keyHandler, ClientKeys);
    else
      moveHandler = new MoveRobot(clients[i], &keyHandler, CloneKeys);
    moveClients.push_back(moveHandler);
  }
}

// create connection to receive point cloud data for all clients
void createPCLReceivers(vector<ArClientBase *> &clients,
    			PCLViewer *viewer,
			vector<PCLOutputHandler *> &pclClients,
			vector<HostInfo> &hostsInfo)
{
  PCLOutputHandler *pclHandler = NULL;

  for (unsigned int i = 0; i < clients.size(); i++) {
    pclHandler = new PCLOutputHandler(clients[i], viewer,
		     hostsInfo[i].locationColor,
		     hostsInfo[i].laserColor,
		     hostsInfo[i].transformInfo.xOffset,
		     hostsInfo[i].transformInfo.yOffset,
		     hostsInfo[i].transformInfo.thetaOffset,
		     hostsInfo[i].requestFreq);
    pclClients.push_back(pclHandler);
  }
}

// just start all the clients
void startClients(vector<ArClientBase *> clients)
{
  for (unsigned int i = 0; i < clients.size(); i++) {
    clients[i]->runAsync();
  }
}

// Check for keys pressed on joystick and orientation of the stick
// itself when in manual mode
void checkJoy(ArJoyHandler *joy, const vector<ArClientBase *> &clients)
{
  static unsigned int currClientIndex = 0;
  static ArClientBase *client = clients[currClientIndex];
  static bool manMode = false;
  static double myTransRatio = 0, myRotRatio = 0;
  static int x = 0, y = 0;

  // control previous robot
  if (joy->getButton(4)) {
    if (currClientIndex == 0) 
      currClientIndex = clients.size() - 1;
    else
      currClientIndex--;
    client = clients[currClientIndex];
    echo("joystick controls", client->getRobotName());
  }
  // control next robot
  else if (joy->getButton(5)) {
    currClientIndex++;
    if (currClientIndex >= clients.size()) 
      currClientIndex = 0;
    client = clients[currClientIndex];
    echo("joystick controls", client->getRobotName());
  }
  // stop the robot
  else if (joy->getButton(2)) {
    if (!client->dataExists("stop")) return;
    else cout << "\t" << client->getRobotName() << " stop mode" << endl;

    manMode = false;
    client->requestOnce("stop");
    myTransRatio = 0;
    myRotRatio = 0;
  }
  // robot is automatic
  else if (joy->getButton(3)) {
    if (!client->dataExists("wander")) return;
    else cout << "\t" << client->getRobotName() << " auto mode" << endl;
    manMode = false;
    client->requestOnce("wander");
    myTransRatio = 0;
    myRotRatio = 0;
  }
  // auto all
  else if (joy->getButton(6)) {
    echo("ALL ROBOTS IN AUTO MODE");
    for (unsigned int i = 0; i < clients.size(); i++)
      clients[i]->requestOnce("wander");
  }
  // stop all
  else if (joy->getButton(7)) {
    echo("ALL ROBOTS IN STOP MODE");
    for (unsigned int i = 0; i < clients.size(); i++)
      clients[i]->requestOnce("stop");
  }
  // safe driving
  else if (joy->getButton(8)) {
    if (!client->dataExists("setSafeDrive")) return;
    else cout << "\t" << client->getRobotName() << " safe drive" << endl;

    ArNetPacket packet;
    packet.byteToBuf(1);
    client->requestOnce("setSafeDrive", &packet);
  }
  // unsafe driving
  else if (joy->getButton(9)) {
    if (!client->dataExists("setSafeDrive")) return;
    else cout << "\t" << client->getRobotName() << " unsafe drive" << endl;

    ArNetPacket packet;
    packet.byteToBuf(0);
    client->requestOnce("setSafeDrive", &packet);
  }

  // manually control the robot
  if (joy->getButton(1)) {
    if (!client->dataExists("ratioDrive")) return;
    else if (!manMode) {
      cout << "\t" << client->getRobotName() << " manual mode" << endl;
    }

    // disable other modes and turn on manual mode
    if (!manMode) {
      client->requestOnce("stop");
      myTransRatio = 0;
      myRotRatio = 0;
      manMode = true;
    }

    joy->getAdjusted(&x, &y);
    myRotRatio = -x;
    myTransRatio = y;
    ArNetPacket packet;
    packet.doubleToBuf(myTransRatio);
    packet.doubleToBuf(myRotRatio);
    packet.doubleToBuf(75);

    client->requestOnce("ratioDrive", &packet);
    myTransRatio = 0;
    myRotRatio = 0;
  }
  else {
    if (!client->dataExists("stop")) return;
    else if (manMode) {
      cout << "\t" << client->getRobotName() << " stop mode" << endl;

      manMode = false;
      client->requestOnce("stop");
      myTransRatio = 0;
      myRotRatio = 0;
    }
  }
}

// Display joystick controls
void joyInfoDisplay()
{
  string keyDesc[] = {
    "move up", "move down", "move left", "move right", 
    "auto mode", "drive mode", "stop mode", "select previous robot",
    "select next robot", "auto all robots", "stop all robots",
    "safe drive", "unsafe drive"
  };

  string keyName[] = {
    "JOYSTICK UP", "JOYSTICK DOWN", "JOYSTICK LEFT", "JOYSTICK RIGHT", 
    "BUTTON 3", "TRIGGER", "BUTTON 2", "BUTTON 4", "BUTTON 5",
    "BUTTON 6", "BUTTON 7", "BUTTON 8", "BUTTON 9"
  };

  for (unsigned int i = 0; i < sizeof(keyDesc)/sizeof(keyDesc[0]); i++)
    cout << keyDesc[i] << " = " << keyName[i] << endl;
  cout << endl;
}

// Creates long filename with the given argument 'name'
// prefixed by the word "cloud" and suffixed by time information
string genCloudFileName(const string &prefix, const string &name)
{
  const char SEPARATOR = '_';
  const char DATE_SEPARATOR = '-';
  const char TIME_SEPARATOR = ':';
  time_t seconds = time(NULL);
  struct tm *timeInfo = localtime(&seconds);

  stringstream new_name;
  new_name << prefix << SEPARATOR
  	   << name << SEPARATOR
           << timeInfo->tm_mon + 1 << DATE_SEPARATOR
           << timeInfo->tm_mday << SEPARATOR
	   << timeInfo->tm_hour << TIME_SEPARATOR
	   << timeInfo->tm_min << TIME_SEPARATOR
	   << timeInfo->tm_sec << ".pcd";

  return new_name.str();
}

// Writes each point cloud from the list of laser point clouds and the
// single point cloud for robot position as files to specified folder.
void writeCloudToFile(vector<PCLOutputHandler *> &pclClients)
{
  string fileName = "";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

  for (size_t i = 0; i < pclClients.size(); i++) {
    // generate the cloud file corresponding to robot position
    fileName = genCloudFileName("robot",
	pclClients[i]->getClient()->getHost());
    cloud = pclClients[i]->getRobotCloud();
    pcl::io::savePCDFile(outputFolder + fileName, *cloud);
    echo("NEW PCD FILE", fileName);

    // generate cloud files corresponding to the time stamped cloud files
    std::vector<TimeStampedPCL *> *laserClouds = 
      pclClients[i]->getLaserClouds();
    int j = 0;
    for (std::vector<TimeStampedPCL *>::const_iterator it =
	 laserClouds->begin(); it != laserClouds->end(); it++) {
      cloud = (*it)->getCloud();
      ostringstream os;
      os << j++;
      fileName = genCloudFileName("laser" + os.str(),
	  pclClients[i]->getClient()->getHost());
      pcl::io::savePCDFile(outputFolder + fileName, *cloud);
      echo("NEW PCD FILE", fileName);
    }
  }
}


// main main
int main(int argc, char **argv)
{
  bool joySupport = false;
  // list of information about each host
  vector<HostInfo> hostsInfo;
  // list of clients to connect to each server
  vector<ArClientBase *> clients;
  // list of input handler objects for clients
  vector<MoveRobot *> moveClients;
  // list of output handler objects for clients
  vector<PCLOutputHandler *> pclClients;

  // needed to initialize aria framework
  Aria::init();
  // parser for command line arguments
  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed()) {
    Aria::shutdown();
    exit(1);
  }

  // This class reads the file and populates the necessary lists
  ConfigFileReader configFileReader(argc, argv, &parser);
  // fill the vectors with information from file
  configFileReader.readHostsFile(hostsInfo);

  // create a connection to each server
  connectHosts(clients, hostsInfo);

  // create the viewer which will show the point cloud
  PCLViewer viewer("Cloud Viewer c2012 FRCV");

  // create the keyhandler which allows manipulating the robots
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  // For safety let the ESC key exist the clients
  ArGlobalFunctor escapeftr(&escapePressed);
  keyHandler.addKeyHandler(ArKeyHandler::ESCAPE, &escapeftr);

  // give each client keys for controlling server robot
  createMovementControls(clients, keyHandler, moveClients);

  // joystick support for client one
  ArJoyHandler joyHandler;
  joySupport = joyHandler.init();
  if (!joySupport) {
    echo("Could not initialize joystick");
  }
  else {
    joyHandler.setSpeeds(50, 100);
    cout << clients[0]->getRobotName() << " joystick controls\n";
    joyInfoDisplay();
  }

  // start all the clients
  startClients(clients);
  // each client will request PCL data
  createPCLReceivers(clients, &viewer, pclClients, hostsInfo);

  // a pointer to one of the clients needed for continuous running
  // of client program
  ArClientBase *client = clients[0];

  // Functor for handling creation of point cloud file
  // the function appends '.pcd' extension
  ArGlobalFunctor1< vector<PCLOutputHandler *>& >
    writeToFileFtr(writeCloudToFile, pclClients);

  keyHandler.addKeyHandler('f', &writeToFileFtr);
  echo("PRESS F TO WRITE POINT CLOUDS TO output/");

  /*
  client->logDataList();  // prints available data on server
  client->logTracking(true); // prints packet transfer info
  */

  // breathing time for inital setup procedures
  ArUtil::sleep(500);

  // Continally check the keyboard presses.
  while (client->getRunningWithLock()) {
    keyHandler.checkKeys();
    // 1st client gets joystick handling
    if (joySupport) checkJoy(&joyHandler, clients);
    // cycle through all clients and check if they are in drive mode
    // if so then send drive instructions
    for (unsigned int i = 0; i < clients.size(); i++) {
      if (moveClients[i]->manMode) moveClients[i]->sendInput();
    }
    ArUtil::sleep(100);
  }

  Aria::shutdown();
}
