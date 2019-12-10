#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensorbearing.h>
#include <rangesensor.h>

using namespace arpro;
using namespace std;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  Robot robot("BB8", 0, 0, 0);
  //Robot robotFollower("Follower", 0, 0, 0);

  RangeSensor rg1(robot, 0.1, 0, 0);
  RangeSensor rg2(robot, 0, 0.1, 0);
  RangeSensor rg3(robot, -0.1, 0, 0);
  RangeSensor rg4(robot, 0, -0.1, 0);

  //SensorBearing sb1(robotFollower, 0.1, 0, 0);

  robot.initWheels(0.3, 0.07, 10);
  // robotFollower.initWheels(0.3, 0.07, 10);

  envir.addRobot(robot);
  // envir.addRobot(robotFollower);

  // simulate 100 sec
  while (envir.time() < 100)
  {
    cout << "---------------------" << endl;
    // robotFollower.moveWithSensor(Twist(0.4, 0, 0));

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
