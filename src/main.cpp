#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>

using namespace std;

int main(int argc, char **argv)
{

  // default environment with moving target
  arpro::Environment envir;
  // sensors gets measurements from this environment
  arpro::Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  arpro::Robot robot("BB8", 0, 0, 0);
  envir.addRobot(robot);

  // simulate 100 sec
  while (envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());
  }

  // plot trajectory
  envir.plot();
}
