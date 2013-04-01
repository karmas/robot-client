#ifndef MOVE_HANDLER_H
#define MOVE_HANDLER_H

#include "ArNetworking.h"

// Abstract base class to provide an interface for
// moving client robots
class MoveHandler {
public:
  // Holds robot servers state information
  struct Mode {
    Mode(bool w, bool s)
      : myWander(w), mySafe(s) { }
    bool myWander;
    bool mySafe;
  };

  virtual void update() = 0;
  virtual void displayKeys() = 0;

protected:
  MoveHandler(std::vector<ArClientBase *> &clients);
  virtual void ratioDrive();
  virtual void wander();
  virtual void stop();
  virtual void safeDrive();
  virtual void nextRobot();
  virtual void prevRobot();
  virtual void wanderAll();
  virtual void stopAll();

  static const char *actions[];

  std::vector<ArClientBase *> &myClients;
  unsigned short myClientIndex;
  ArClientBase *myClient;
  double myTransRatio;
  double myRotRatio;
  double mySpeedLimit;
  std::vector<Mode> myModes;
};

// Move using keyboard
class MoveKeyHandler : public MoveHandler {
public:
  MoveKeyHandler(std::vector<ArClientBase *> &clients,
      ArKeyHandler *keyHandler);
  virtual void update();
  virtual void displayKeys();

private:
  virtual void nextRobot();
  virtual void prevRobot();
  void forward();
  void backward();
  void turnLeft();
  void turnRight();

  static const int keys[];

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
  ArFunctorC<MoveKeyHandler> myWanderAllFtr;
  ArFunctorC<MoveKeyHandler> myStopAllFtr;
};

// Move using Joystick
class MoveJoyHandler : public MoveHandler {
public:
  MoveJoyHandler(std::vector<ArClientBase *> &clients,
      ArJoyHandler *joyHandler);
  virtual void update();
  virtual void displayKeys();

private:
  virtual void ratioDrive();
  virtual void nextRobot();
  virtual void prevRobot();

  static const char *keys[];

  ArJoyHandler *myJoyHandler;
};



// useful movement related function declarations
void defaultMoveKeys(std::vector<int> &moveKeys, 
    std::vector<std::string> &moveKeysInfo);
std::string moveKeyToString(int c);
void keyVsJoy(MoveHandler *&key, MoveHandler *&joy, int nClients);


#endif
