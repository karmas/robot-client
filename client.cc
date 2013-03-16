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
 * Press 'p' to create a pcd file of the current point cloud
 */

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>

#include "Aria.h"

#include "helpers.h"
#include "ConfigFileReader.h"
#include "MoveRobot.h"
#include "SensorDataHandler.h"
#include "SensorDataViewer.h"


// main main
int main(int argc, char **argv)
{
  bool joySupport = false;
  // list of information about each host
  std::vector<HostInfo> hostsInfo;
  // list of clients to connect to each server
  std::vector<ArClientBase *> clients;
  // list of sensor data handlers for clients
  std::vector<SensorDataHandler *> sensorDataHandlers;
  // list of output handler objects for clients
  std::vector<PCLOutputHandler *> pclClients;

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

  // create the keyhandler which allows manipulating the robots
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  // For safety let the ESC key exist the clients
  ArGlobalFunctor escapeftr(&escapePressed);
  keyHandler.addKeyHandler(ArKeyHandler::ESCAPE, &escapeftr);

  // keyboard movement controls
  MoveRobot moveRobot(&keyHandler, clients);

  // joystick support for client one
  ArJoyHandler joyHandler;
  joySupport = joyHandler.init();
  if (!joySupport) {
    echo("Could not initialize joystick");
  }
  else {
    joyHandler.setSpeeds(50, 100);
    std::cout << clients[0]->getRobotName() << " joystick controls\n";
    joyInfoDisplay();
  }

  // start all the clients
  startClients(clients);
  // create the sensor data handlers for the clients
  createSensorDataHandlers(clients, sensorDataHandlers, hostsInfo);
  SensorDataViewer viewer("SensorDataViewer FRCV", sensorDataHandlers);

  // a pointer to one of the clients needed for continuous running
  // of client program
  ArClientBase *client = clients[0];

  // Functor for handling creation of point cloud file
  // the function appends '.pcd' extension
  ArGlobalFunctor1< std::vector<PCLOutputHandler *>& >
    writeToFileFtr(writeCloudToFile, pclClients);

  keyHandler.addKeyHandler('p', &writeToFileFtr);
  echo("PRESS P IN TERMINAL TO WRITE POINT CLOUDS");

  // Functor for handling initiation of data transfer
  ArGlobalFunctor1< std::vector<PCLOutputHandler *>& >
    beginDataTransferFtr(beginDataTransfer, pclClients);

  keyHandler.addKeyHandler('b', &beginDataTransferFtr);
  echo("PRESS B TO BEGIN DATA TRANSFER FROM ROBOT SERVERS");

  // breathing time for inital setup procedures
  ArUtil::sleep(500);

  // Continally check the keyboard presses.
  while (client->getRunningWithLock()) {
    keyHandler.checkKeys();
    // 1st client gets joystick handling
    if (joySupport) checkJoy(&joyHandler, clients);
    if (moveRobot.manMode) moveRobot.sendInput();
    ArUtil::sleep(100);

    // refresh the viewer
    viewer.updateDisplay();
  }

  Aria::shutdown();
}
