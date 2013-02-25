#include <iostream>
#include <iomanip>

#include "MoveRobot.h"
#include "helpers.h"

// these keys control the movement of the robot
int MoveRobot::moveKeys[] = { 
  ArKeyHandler::UP,
  ArKeyHandler::DOWN,
  ArKeyHandler::LEFT,
  ArKeyHandler::RIGHT,
  'a',
  'd',
  's',
  ArKeyHandler::PAGEUP,
  ArKeyHandler::PAGEDOWN,
  'q',
  'w',
  'f',
  'g',
};
// information on what kind of movement the corresponding key performs
const char *MoveRobot::moveKeysInfo[] = {
  "move forward",
  "move backward",
  "rotate left",
  "rotate right",
  "auto mode",
  "drive mode",
  "stop mode",
  "control previous robot",
  "control next robot",
  "all auto mode",
  "all stop mode",
  "safe drive",
  "unsafe drive",
};

MoveRobot::MoveRobot(ArKeyHandler *keyHandler,
    		     std::vector<ArClientBase *> &clients)
  : myKeyHandler(keyHandler), myClients(clients),
    myClient(clients[0]), myClientIndex(0),
    manMode(false), myTransRatio(0.0), myRotRatio(0.0),
    upftr(this, &MoveRobot::up),
    downftr(this, &MoveRobot::down),
    leftftr(this, &MoveRobot::left),
    rightftr(this, &MoveRobot::right),
    autoMoveftr(this, &MoveRobot::autoMove),
    manMoveftr(this, &MoveRobot::manMove),
    stopMoveftr(this, &MoveRobot::stopMove),
    prevRobotftr(this, &MoveRobot::prevRobot),
    nextRobotftr(this, &MoveRobot::nextRobot),
    allAutoMoveftr(this, &MoveRobot::allAutoMove),
    allStopMoveftr(this, &MoveRobot::allStopMove),
    safeDriveftr(this, &MoveRobot::safeDrive),
    unSafeDriveftr(this, &MoveRobot::unSafeDrive)
{
  myKeyHandler->addKeyHandler(moveKeys[0], &upftr);
  myKeyHandler->addKeyHandler(moveKeys[1], &downftr);
  myKeyHandler->addKeyHandler(moveKeys[2], &leftftr);
  myKeyHandler->addKeyHandler(moveKeys[3], &rightftr);
  myKeyHandler->addKeyHandler(moveKeys[4], &autoMoveftr);
  myKeyHandler->addKeyHandler(moveKeys[5], &manMoveftr);
  myKeyHandler->addKeyHandler(moveKeys[6], &stopMoveftr);
  myKeyHandler->addKeyHandler(moveKeys[7], &prevRobotftr);
  myKeyHandler->addKeyHandler(moveKeys[8], &nextRobotftr);
  myKeyHandler->addKeyHandler(moveKeys[9], &allAutoMoveftr);
  myKeyHandler->addKeyHandler(moveKeys[10], &allStopMoveftr);
  myKeyHandler->addKeyHandler(moveKeys[11], &safeDriveftr);
  myKeyHandler->addKeyHandler(moveKeys[12], &unSafeDriveftr);
  displayKeys();
}

// Request the robots to stop 
MoveRobot::~MoveRobot()
{
  myClient->requestOnce("stop");
}

static std::string moveKeyToString(int c)
{
  std::string keyName;
  switch (c) {
    case ArKeyHandler::UP:
      keyName = "UP arrow";
      break;
    case ArKeyHandler::DOWN:
      keyName = "DOWN arrow";
      break;
    case ArKeyHandler::LEFT:
      keyName = "LEFT arrow";
      break;
    case ArKeyHandler::RIGHT:
      keyName = "RIGHT arrow";
      break;
    case ArKeyHandler::PAGEUP:
      keyName = "PAGEUP arrow";
      break;
    case ArKeyHandler::PAGEDOWN:
      keyName = "PAGEDOWN arrow";
      break;
    default:
      keyName = char(c);
      break;
  }
  return keyName;
}

// Tell the user about the control keys
void MoveRobot::displayKeys() 
{
  const int leftMargin = 5;
  const int keyColWidth = 20;
  const int descColWidth = 30;

  std::cout << std::endl << "Keyboard controls" << std::endl;
  std::cout << std::setw(leftMargin) << "|"
    << std::setw(keyColWidth) << "Key" 
    << std::setw(descColWidth) << "Description" 
    << std::endl
    << std::string(60, '-')
    << std::endl;

  for (int i = 0; i < sizeof(moveKeys)/sizeof(moveKeys[0]); i++) {
  std::cout << std::setw(leftMargin) << "|"
    << std::setw(keyColWidth) << moveKeyToString(moveKeys[i])
    << std::setw(descColWidth) << moveKeysInfo[i]
    << std::endl;
  }

  std::cout << std::endl;
}

void MoveRobot::up() { myTransRatio = 100; }
void MoveRobot::down() { myTransRatio = -100; }
void MoveRobot::left() { myRotRatio = 100; }
void MoveRobot::right() { myRotRatio = -100; }

// sets server robot to wander
void MoveRobot::autoMove()
{
  // check if server supports automatic movement
  if (!myClient->dataExists("wander")) return;
  else std::cout << "\t" << myClient->getRobotName() 
    << " auto mode" << std::endl;

  // disable driving mode
  manMode = false;
  // ask server to wander
  myClient->requestOnce("wander");
  myTransRatio = 0;
  myRotRatio = 0;
}

// sets server robot to manual drive
void MoveRobot::manMove()
{
  if (!myClient->dataExists("ratioDrive")) return;
  else std::cout << "\t" << myClient->getRobotName() 
    << " manual mode" << std::endl;

  manMode = true;
}

// stops the server robot from moving
void MoveRobot::stopMove()
{
  if (!myClient->dataExists("stop")) return;
  else std::cout << "\t" << myClient->getRobotName() 
    << " stop mode" << std::endl;

  manMode = false;
  myClient->requestOnce("stop");
  myTransRatio = 0;
  myRotRatio = 0;
}

// sends a packet with velocity information based on move commands
void MoveRobot::sendInput()
{
  if (!myClient->dataExists("ratioDrive")) return;

  ArNetPacket packet;
  packet.doubleToBuf(myTransRatio);
  packet.doubleToBuf(myRotRatio);
  packet.doubleToBuf(50);

  myClient->requestOnce("ratioDrive", &packet);
  myTransRatio = 0;
  myRotRatio = 0;
}

// select the previous robot in case of multiple robots
void MoveRobot::prevRobot()
{
  if (myClientIndex == 0) 
    myClientIndex = myClients.size() - 1;
  else
    myClientIndex--;
  myClient = myClients[myClientIndex];
  echo("keyboard controls", myClient->getRobotName());
}

// select the next robot in case of multiple robots
void MoveRobot::nextRobot()
{
  myClientIndex++;
  if (myClientIndex >= myClients.size()) 
    myClientIndex = 0;
  myClient = myClients[myClientIndex];
  echo("keyboard controls", myClient->getRobotName());
}

// all robots in automatic mode
void MoveRobot::allAutoMove()
{
  echo("ALL ROBOTS IN AUTO MODE");
  for (unsigned int i = 0; i < myClients.size(); i++)
    myClients[i]->requestOnce("wander");
}

// all robots in stop mode
void MoveRobot::allStopMove()
{
  echo("ALL ROBOTS IN STOP MODE");
  for (unsigned int i = 0; i < myClients.size(); i++)
    myClients[i]->requestOnce("stop");
}

// safe driving robot
void MoveRobot::safeDrive()
{
  if (!myClient->dataExists("setSafeDrive")) return;
  else std::cout << "\t" << myClient->getRobotName() 
    << " safe drive" << std::endl;

  ArNetPacket packet;
  packet.byteToBuf(1);
  myClient->requestOnce("setSafeDrive", &packet);
}

// not a safe driving robot
void MoveRobot::unSafeDrive()
{
  if (!myClient->dataExists("setSafeDrive")) return;
  else std::cout << "\t" << myClient->getRobotName() 
    << " unsafe drive" << std::endl;

  ArNetPacket packet;
  packet.byteToBuf(0);
  myClient->requestOnce("setSafeDrive", &packet);
}



// Check for keys pressed on joystick and orientation of the stick
// itself when in manual mode
void checkJoy(ArJoyHandler *joy, const std::vector<ArClientBase *> &clients)
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
    else std::cout << "\t" << client->getRobotName() 
      << " stop mode" << std::endl;

    manMode = false;
    client->requestOnce("stop");
    myTransRatio = 0;
    myRotRatio = 0;
  }
  // robot is automatic
  else if (joy->getButton(3)) {
    if (!client->dataExists("wander")) return;
    else std::cout << "\t" << client->getRobotName() 
      << " auto mode" << std::endl;
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
    else std::cout << "\t" << client->getRobotName() 
      << " safe drive" << std::endl;

    ArNetPacket packet;
    packet.byteToBuf(1);
    client->requestOnce("setSafeDrive", &packet);
  }
  // unsafe driving
  else if (joy->getButton(9)) {
    if (!client->dataExists("setSafeDrive")) return;
    else std::cout << "\t" << client->getRobotName() 
      << " unsafe drive" << std::endl;

    ArNetPacket packet;
    packet.byteToBuf(0);
    client->requestOnce("setSafeDrive", &packet);
  }

  // manually control the robot
  if (joy->getButton(1)) {
    if (!client->dataExists("ratioDrive")) return;
    else if (!manMode) {
      std::cout << "\t" << client->getRobotName() 
	<< " manual mode" << std::endl;
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
      std::cout << "\t" << client->getRobotName() 
	<< " stop mode" << std::endl;

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
  std::string keyDesc[] = {
    "move up", "move down", "move left", "move right", 
    "auto mode", "drive mode", "stop mode", "select previous robot",
    "select next robot", "auto all robots", "stop all robots",
    "safe drive", "unsafe drive"
  };

  std::string keyName[] = {
    "JOYSTICK UP", "JOYSTICK DOWN", "JOYSTICK LEFT", "JOYSTICK RIGHT", 
    "BUTTON 3", "TRIGGER", "BUTTON 2", "BUTTON 4", "BUTTON 5",
    "BUTTON 6", "BUTTON 7", "BUTTON 8", "BUTTON 9"
  };

  for (unsigned int i = 0; i < sizeof(keyDesc)/sizeof(keyDesc[0]); i++)
    std::cout << keyDesc[i] << " = " << keyName[i] << std::endl;
  std::cout << std::endl;
}
