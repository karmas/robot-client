#ifndef MOVE_HANDLER_H
#define MOVE_HANDLER_H

#include "ArNetworking.h"

// Abstract base class to provide an interface for
// moving client robots
class MoveHandler {
public:
  virtual void update() = 0;
  virtual void displayKeys() = 0;

protected:
  MoveHandler(std::vector<ArClientBase *> &clients,
      std::vector<int> &keys, std::vector<std::string> &keysInfo);
  virtual void forward() = 0;
  virtual void backward() = 0;
  virtual void turnLeft() = 0;
  virtual void turnRight() = 0;
  virtual void wander() = 0;
  virtual void stop() = 0;
  virtual void unsafe() = 0;
  virtual void nextRobot() = 0;
  virtual void prevRobot() = 0;

  std::vector<ArClientBase *> &myClients;
  std::vector<int> &myKeys;
  std::vector<std::string> &myKeysInfo;
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
      std::vector<int> &keys, std::vector<std::string> &keysInfo,
      ArKeyHandler *keyHandler);
  void update();
  void displayKeys();

private:
  void forward();
  void backward();
  void turnLeft();
  void turnRight();
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



// useful movement related function declarations
void checkJoy(ArJoyHandler *joy, const std::vector<ArClientBase *> &clients);
void joyInfoDisplay();

#endif
