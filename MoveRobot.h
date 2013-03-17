#ifndef MOVE_ROBOT_H
#define MOVE_ROBOT_H

#include "ArNetworking.h"

// Abstract base class to provide an interface for
// moving client robots
class MoveHandler {
public:
  virtual void update() = 0;
  virtual void forward() = 0;
  virtual void backward() = 0;
  virtual void turnLeft() = 0;
  virtual void turnRight() = 0;
  virtual void ratioDrive() = 0;
  virtual void wander() = 0;
  virtual void stop() = 0;
  virtual void unsafe() = 0;
  virtual void nextRobot() = 0;
  virtual void prevRobot() = 0;

protected:
  MoveHandler(std::vector<ArClientBase *> &clients);

  std::vector<ArClientBase *> &myClients;
  int myClientIndex;
  ArClientBase *myClient;
  double myTransRatio;
  double myRotRatio;
  double mySpeedLimit;
  bool myIsWandering;
  bool myIsSafe;
};

// Move using keyboard
class MoveKeyBoardHandler : public MoveHandler {
public:
  MoveKeyBoardHandler(std::vector<ArClientBase *> &clients,
      ArKeyHandler *keyHandler);

private:
  void update();
  void forward();
  void backward();
  void turnLeft();
  void turnRight();
  void ratioDrive();
  void wander();
  void stop();
  void unsafe();
  void nextRobot();
  void prevRobot();

  ArKeyHandler *myKeyHandler;
  ArFunctorC<MoveKeyBoardHandler> myForwardFtr;
  ArFunctorC<MoveKeyBoardHandler> myBackwardFtr;
  ArFunctorC<MoveKeyBoardHandler> myTurnLeftFtr;
  ArFunctorC<MoveKeyBoardHandler> myTurnRightFtr;
  ArFunctorC<MoveKeyBoardHandler> myWanderFtr;
  ArFunctorC<MoveKeyBoardHandler> myStopFtr;
  ArFunctorC<MoveKeyBoardHandler> myUnsafeFtr;
  ArFunctorC<MoveKeyBoardHandler> myNextRobotFtr;
  ArFunctorC<MoveKeyBoardHandler> myPrevRobotFtr;
};

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
