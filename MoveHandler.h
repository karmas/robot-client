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
class MoveKeyHandler : public MoveHandler {
public:
  MoveKeyHandler(std::vector<ArClientBase *> &clients,
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
  ArFunctorC<MoveKeyHandler> myForwardFtr;
  ArFunctorC<MoveKeyHandler> myBackwardFtr;
  ArFunctorC<MoveKeyHandler> myTurnLeftFtr;
  ArFunctorC<MoveKeyHandler> myTurnRightFtr;
  ArFunctorC<MoveKeyHandler> myWanderFtr;
  ArFunctorC<MoveKeyHandler> myStopFtr;
  ArFunctorC<MoveKeyHandler> myUnsafeFtr;
  ArFunctorC<MoveKeyHandler> myNextRobotFtr;
  ArFunctorC<MoveKeyHandler> myPrevRobotFtr;
};

// Move using Joystick
class MoveJoyHandler : public MoveHandler {
public:
  MoveJoyHandler(std::vector<ArClientBase *> &clients,
      std::vector<int> &keys, std::vector<std::string> &keyInfo,
      ArJoyHandler *joyHandler);
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

  ArJoyHandler *myJoyHandler;
};



// useful movement related function declarations
void defaultMoveKeys(std::vector<int> &moveKeys, 
    std::vector<std::string> &moveKeysInfo);
std::string moveKeyToString(int c);
void checkJoy(ArJoyHandler *joy, const std::vector<ArClientBase *> &clients);
void joyInfoDisplay();

#endif
