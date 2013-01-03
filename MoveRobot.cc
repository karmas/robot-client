#include "MoveRobot.h"
#include <iostream>
using namespace std;


MoveRobot::MoveRobot(ArClientBase *client, ArKeyHandler *keyHandler,
                     int *moveKeys)
  : myClient(client), myKeyHandler(keyHandler), myMoveKeys(moveKeys),
    manMode(false), myTransRatio(0.0), myRotRatio(0.0),
    upftr(this, &MoveRobot::up),
    downftr(this, &MoveRobot::down),
    leftftr(this, &MoveRobot::left),
    rightftr(this, &MoveRobot::right),
    autoMoveftr(this, &MoveRobot::autoMove),
    manMoveftr(this, &MoveRobot::manMove),
    stopMoveftr(this, &MoveRobot::stopMove)
{
  myKeyHandler->addKeyHandler(moveKeys[0], &upftr);
  myKeyHandler->addKeyHandler(moveKeys[1], &downftr);
  myKeyHandler->addKeyHandler(moveKeys[2], &leftftr);
  myKeyHandler->addKeyHandler(moveKeys[3], &rightftr);
  myKeyHandler->addKeyHandler(moveKeys[4], &autoMoveftr);
  myKeyHandler->addKeyHandler(moveKeys[5], &manMoveftr);
  myKeyHandler->addKeyHandler(moveKeys[6], &stopMoveftr);
  displayKeys();
}

// Request the robots to stop 
MoveRobot::~MoveRobot()
{
  myClient->requestOnce("stop");
}

// Tell the user about the control keys
void MoveRobot::displayKeys() 
{
  string keyDesc[] = {
    "move up", "move down", "move left", "move right", 
    "auto mode", "drive mode", "stop mode"
  };

  string keyName[] = {
    "UP", "DOWN", "LEFT", "RIGHT", "ESCAPE", "SPACE"
  };

  cout << endl << myClient->getRobotName() << " control keys" << endl;

  for (int i = 0; i < myNumKeys; i++) {
    int c = myMoveKeys[i];
    if (c < 256)
      cout << keyDesc[i] << " = " << (char) c << endl;
    else
      cout << keyDesc[i] << " = " << keyName[c - 256] << endl;
  }

  cout << endl;
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
  else cout << "\t" << myClient->getRobotName() << " auto mode" << endl;

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
  else cout << "\t" << myClient->getRobotName() << " manual mode" << endl;

  manMode = true;
}

// stops the server robot from moving
void MoveRobot::stopMove()
{
  if (!myClient->dataExists("stop")) return;
  else cout << "\t" << myClient->getRobotName() << " stop mode" << endl;

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
