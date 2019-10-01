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

/* Q1 : the target motion is defined in the envir class
 * Q2 : It's the constructor of the Robot class, it constructs a new robot with the parameters
 * passed as arguments. 
 * You can't modify the parameters afterwards because you can't access them inside the class.
 * Q4 : To make it impossible to use outside the Robot class, we can put the method as protected or private.
 * Private will provent any other class from accessing the method.
 * Protected will provent any other class that aren't children to the Robot class to access it.
 */
