#ifndef MOVE_ROBOT_H
#define MOVE_ROBOT_H

#include "ArNetworking.h"

// This class houses the functions which handle keyboard events
// for setting the various robot modes and driving the robot
// The keys can be defined by the user as an array of characters.
// Below is the description of the keys
// moveKeys [] = { up, down, left, right, auto, manual, stop }
class MoveRobot {
public:
  MoveRobot(ArKeyHandler *keyHandler, 
            std::vector<ArClientBase *> &clients);
  ~MoveRobot();
  void displayKeys();
  void up();
  void down();
  void left();
  void right();
  void autoMove();
  void manMove();
  void stopMove();
  void prevRobot();
  void nextRobot();
  void allAutoMove();
  void allStopMove();
  void safeDrive();
  void unSafeDrive();
  void sendInput();

  static int moveKeys[];
  static const char *moveKeysInfo[];

  ArKeyHandler *myKeyHandler;
  std::vector<ArClientBase *> &myClients;

  bool manMode;
  ArClientBase *myClient;
  int myClientIndex;
  double myTransRatio;
  double myRotRatio;

  ArFunctorC<MoveRobot> upftr;
  ArFunctorC<MoveRobot> downftr;
  ArFunctorC<MoveRobot> leftftr;
  ArFunctorC<MoveRobot> rightftr;
  ArFunctorC<MoveRobot> autoMoveftr;
  ArFunctorC<MoveRobot> manMoveftr;
  ArFunctorC<MoveRobot> stopMoveftr;
  ArFunctorC<MoveRobot> prevRobotftr;
  ArFunctorC<MoveRobot> nextRobotftr;
  ArFunctorC<MoveRobot> allAutoMoveftr;
  ArFunctorC<MoveRobot> allStopMoveftr;
  ArFunctorC<MoveRobot> safeDriveftr;
  ArFunctorC<MoveRobot> unSafeDriveftr;

  static void chooseMoveKeys();
};


// useful movement related function declarations
void checkJoy(ArJoyHandler *joy, const std::vector<ArClientBase *> &clients);
void joyInfoDisplay();

#endif
