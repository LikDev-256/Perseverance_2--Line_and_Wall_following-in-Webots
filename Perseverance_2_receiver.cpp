#include <webots/Robot.hpp>
#include <webots/Receiver.hpp>
#include <webots/LED.hpp>

#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdio.h>
#include<bits/stdc++.h>

using namespace webots;
using namespace std;

  Robot *robot = new Robot();

  Receiver *receiver = robot -> getReceiver("receiver");

  LED *RED  = robot -> getLED("led_red");
  LED *BLUE = robot -> getLED("led_blue");

  int timeStep = (int)robot->getBasicTimeStep();

int main(int argc, char **argv) {

  while (robot->step(timeStep) != -1) {

    receiver->enable(timeStep);

    if (receiver->getQueueLength() > 0) {
      string message((const char *)receiver->getData());
      receiver->nextPacket();

      //cout<<message<<endl;
      if (message.compare("R") == 0) {
        RED  -> set(1);
        BLUE -> set(0);
      }

      if (message.compare("B") == 0) {
        BLUE -> set(3);
        RED  -> set(0);
      }
    }

  };

  delete robot;
  return 0;
}
