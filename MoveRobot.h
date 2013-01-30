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
  MoveRobot(ArClientBase *client, ArKeyHandler *keyHandler,
            int *moveKeys);
  ~MoveRobot();
  void displayKeys();
  void up();
  void down();
  void left();
  void right();
  void autoMove();
  void manMove();
  void stopMove();
  void sendInput();

  static const int myNumKeys = 7;

  ArClientBase *myClient;
  ArKeyHandler *myKeyHandler;
  int *myMoveKeys;

  bool manMode;
  double myTransRatio;
  double myRotRatio;

  ArFunctorC<MoveRobot> upftr;
  ArFunctorC<MoveRobot> downftr;
  ArFunctorC<MoveRobot> leftftr;
  ArFunctorC<MoveRobot> rightftr;
  ArFunctorC<MoveRobot> autoMoveftr;
  ArFunctorC<MoveRobot> manMoveftr;
  ArFunctorC<MoveRobot> stopMoveftr;
};


// useful movement related function declarations
void createMovementControls(std::vector<ArClientBase *> &clients, 
                            ArKeyHandler &keyHandler,
                            std::vector<MoveRobot *> &moveClients);
void checkJoy(ArJoyHandler *joy, const std::vector<ArClientBase *> &clients);
void joyInfoDisplay();

#endif
