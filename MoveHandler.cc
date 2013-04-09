#include <iostream>
#include <iomanip>

#include "MoveHandler.h"
#include "helpers.h"

// names of actions
const char *MoveHandler::actions[] = {
  "Move forward",
  "Move backward",
  "Turn left",
  "Turn right",
  "Wander",
  "Stop",
  "Unsafe on/off",
  "Next robot",
  "Previous robot",
  "Wander All",
  "Stop All",
};

MoveHandler::MoveHandler(std::vector<ArClientBase *> &clients)
  : myClients(clients), myClientIndex(0), myClient(myClients[0]),
    myTransRatio(0), myRotRatio(0), mySpeedLimit(50)
{
}

void MoveHandler::setClientIndex(size_t i)
{
  if (i < myClients.size()) {
    myClientIndex = i;
    myClient = myClients[myClientIndex];
  }
}

// send a packet containing velocity values
void MoveHandler::ratioDrive() 
{ 
  if (!myClient->dataExists("ratioDrive") ||
      (*robotModes)[myClientIndex].myWander) return;

  static ArNetPacket packet;
  packet.empty();
  packet.doubleToBuf(myTransRatio);
  packet.doubleToBuf(myRotRatio);
  packet.doubleToBuf(mySpeedLimit);

  myClient->requestOnce("ratioDrive", &packet);
  myTransRatio = 0;
  myRotRatio = 0;
}

// start wandering
void MoveHandler::wander()
{
  if (!myClient->dataExists("wander") ||
      (*robotModes)[myClientIndex].myWander) return;

  (*robotModes)[myClientIndex].myWander = true;
  myClient->requestOnce("wander");
  std::cout << "\t" << myClient->getRobotName() 
    << " wander mode" << std::endl;
}

// stop the robot
void MoveHandler::stop()
{
  if (!myClient->dataExists("stop")) return;
  myClient->requestOnce("stop");
  (*robotModes)[myClientIndex].myWander = false;
}

// alternate between unsafe and safe drive
void MoveHandler::safeDrive()
{
  if (!myClient->dataExists("setSafeDrive")) return;

  (*robotModes)[myClientIndex].mySafe = 
    !(*robotModes)[myClientIndex].mySafe;

  ArNetPacket packet;
  if ((*robotModes)[myClientIndex].mySafe) {
    packet.byteToBuf(1);
    std::cout << "\t" << myClient->getRobotName() 
      << " safe drive" << std::endl;
  }
  else {
    packet.byteToBuf(0);
    std::cout << "\t" << myClient->getRobotName() 
      << " unsafe drive" << std::endl;
  }
  myClient->requestOnce("setSafeDrive", &packet);
}

// control next robot
void MoveHandler::nextRobot()
{
  myClientIndex++;
  if (myClientIndex >= myClients.size()) myClientIndex = 0;
  myClient = myClients[myClientIndex];
}

// control previous robot
void MoveHandler::prevRobot()
{
  if (myClientIndex == 0) myClientIndex = myClients.size() - 1;
  else myClientIndex--;
  myClient = myClients[myClientIndex];
}

// all robots wander
void MoveHandler::wanderAll()
{
  for (size_t i = 0; i < myClients.size(); i++) {
    myClients[i]->requestOnce("wander");
    (*robotModes)[i].myWander = true;
  }
}

// all robots stop
void MoveHandler::stopAll()
{
  for (size_t i = 0; i < myClients.size(); i++) {
    myClients[i]->requestOnce("stop");
    (*robotModes)[i].myWander = false;
  }
}


////////////////////////////////////////////
//   MoveKeyHandler
////////////////////////////////////////////

// fill the command keys
const int MoveKeyHandler::keys[] = {
  ArKeyHandler::UP,
  ArKeyHandler::DOWN,
  ArKeyHandler::LEFT,
  ArKeyHandler::RIGHT,
  'w',
  's',
  'x',
  ArKeyHandler::PAGEUP,
  ArKeyHandler::PAGEDOWN,
  'q',
  'a',
};

// Attach keypress handlers
MoveKeyHandler::MoveKeyHandler(
    std::vector<ArClientBase *> &clients,
    ArKeyHandler *keyHandler)
  : MoveHandler(clients), 
    myKeyHandler(keyHandler),
    myForwardFtr(this, &MoveKeyHandler::forward),
    myBackwardFtr(this, &MoveKeyHandler::backward),
    myTurnLeftFtr(this, &MoveKeyHandler::turnLeft),
    myTurnRightFtr(this, &MoveKeyHandler::turnRight),
    myWanderFtr(this, &MoveKeyHandler::wander),
    myStopFtr(this, &MoveKeyHandler::stop),
    myUnsafeFtr(this, &MoveKeyHandler::safeDrive),
    myNextRobotFtr(this, &MoveKeyHandler::nextRobot),
    myPrevRobotFtr(this, &MoveKeyHandler::prevRobot),
    myWanderAllFtr(this, &MoveKeyHandler::wanderAll),
    myStopAllFtr(this, &MoveKeyHandler::stopAll)
{
  myKeyHandler->addKeyHandler(keys[0], &myForwardFtr);
  myKeyHandler->addKeyHandler(keys[1], &myBackwardFtr);
  myKeyHandler->addKeyHandler(keys[2], &myTurnLeftFtr);
  myKeyHandler->addKeyHandler(keys[3], &myTurnRightFtr);
  myKeyHandler->addKeyHandler(keys[4], &myWanderFtr);
  myKeyHandler->addKeyHandler(keys[5], &myStopFtr);
  myKeyHandler->addKeyHandler(keys[6], &myUnsafeFtr);
  myKeyHandler->addKeyHandler(keys[7], &myNextRobotFtr);
  myKeyHandler->addKeyHandler(keys[8], &myPrevRobotFtr);
  myKeyHandler->addKeyHandler(keys[9], &myWanderAllFtr);
  myKeyHandler->addKeyHandler(keys[10], &myStopAllFtr);
}

// For keyboard, the key press checking is done
// by the keyhandler so only send the movement
// values
void MoveKeyHandler::update()
{
  ratioDrive();
}

// print commands and keys
void MoveKeyHandler::displayKeys()
{
  printTitle("Keyboard movement controls");

  const int colWidthAction = 20;
  const int colWidthKey = 20;

  std::cout << std::setw(colWidthAction) << "Action"
    << std::setw(colWidthKey) << "Key" << std::endl
    << std::string(colWidthAction + colWidthKey, '-') << std::endl;

  for (size_t i = 0; i < sizeof(keys)/sizeof(keys[0]); i++) {
    std::cout << std::setw(colWidthAction) << actions[i]
      << std::setw(colWidthKey) << moveKeyToString(keys[i]) << std::endl;
  }

  std::cout << std::endl;
}

// just display current robot name
void MoveKeyHandler::nextRobot()
{
  MoveHandler::nextRobot();
  echo("Keyboard controls", myClient->getRobotName());
}

// just display current robot name
void MoveKeyHandler::prevRobot()
{
  MoveHandler::nextRobot();
  echo("Keyboard controls", myClient->getRobotName());
}

// fix movement values
void MoveKeyHandler::forward() { myTransRatio = 100; }
void MoveKeyHandler::backward() { myTransRatio = -100; }
void MoveKeyHandler::turnLeft() { myRotRatio = 100; }
void MoveKeyHandler::turnRight() { myRotRatio = -100; }


////////////////////////////////////////////
//   MoveJoyHandler
////////////////////////////////////////////

// fill the command keys
const char *MoveJoyHandler::keys[] = {
  "Joystick UP",
  "Joystick DOWN",
  "Joystick LEFT",
  "Joystick RIGHT",
  "Button 6",
  "Button 7",
  "Button 3",
  "Button 4",
  "Button 5",
  "Button 11",
  "Button 10",
};

// Since joystick is analog, give high speed limit
MoveJoyHandler::MoveJoyHandler(
    std::vector<ArClientBase *> &clients, ArJoyHandler *joyHandler)
  : MoveHandler(clients), myJoyHandler(joyHandler)
{
  mySpeedLimit = 90;
}

// check for key press and perform appropriate command
void MoveJoyHandler::update()
{
  if (myJoyHandler->getButton(1)) ratioDrive();
  else {
    if (!(*robotModes)[myClientIndex].myWander) {
      myTransRatio = 0;
      myRotRatio = 0;
      MoveHandler::ratioDrive();
    }
    if (myJoyHandler->getButton(6)) wander();
    else if (myJoyHandler->getButton(7)) stop();
    else if (myJoyHandler->getButton(3)) safeDrive();
    else if (myJoyHandler->getButton(4)) prevRobot();
    else if (myJoyHandler->getButton(5)) nextRobot();
    else if (myJoyHandler->getButton(11)) wanderAll();
    else if (myJoyHandler->getButton(10)) stopAll();
  }
}

// print keys and description on command line
void MoveJoyHandler::displayKeys()
{
  printTitle("Joystick movement controls");

  const int colWidthAction = 20;
  const int colWidthKey = 20;

  std::cout << std::setw(colWidthAction) << "Action"
    << std::setw(colWidthKey) << "Key" << std::endl
    << std::string(colWidthAction + colWidthKey, '-') << std::endl;

  for (size_t i = 0; i < sizeof(keys)/sizeof(keys[0]); i++) {
    std::cout << std::setw(colWidthAction) << actions[i]
      << std::setw(colWidthKey) << keys[i] << std::endl;
  }

  std::cout << std::endl;
}

// manually control the robot by getting velocities from
// joystick position
void MoveJoyHandler::ratioDrive()
{
  int x, y;

  myJoyHandler->getAdjusted(&x, &y);
  myRotRatio = -x;
  myTransRatio = y;

  MoveHandler::ratioDrive();
}

// just display current robot name
void MoveJoyHandler::nextRobot()
{
  MoveHandler::nextRobot();
  echo("Joystick controls", myClient->getRobotName());
}

// just display current robot name
void MoveJoyHandler::prevRobot()
{
  MoveHandler::nextRobot();
  echo("Joystick controls", myClient->getRobotName());
}




////////////////////////////////////////////
//   Useful Functions
////////////////////////////////////////////

// return a string name of the given key
std::string moveKeyToString(int c)
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

// @func: Creates a list of robot modes and passes a pointer to that list
//   to each move handler. 
void initControllers(MoveHandler *&key, MoveHandler *&joy, int nClients)
{
  std::vector<Mode> *modes = new std::vector<Mode>;
  for (int i = 0; i < nClients; i++) {
    modes->push_back(Mode(false, true));
  }

  key->displayKeys();
  key->robotModes = modes;
  if (joy) {
    joy->displayKeys();
    joy->robotModes = modes;
    if (nClients > 1) key->setClientIndex(1);
  }

  printTitle("DO NOT CONTROL SAME ROBOT WITH KEYBOARD AND JOYSTICK");
}
