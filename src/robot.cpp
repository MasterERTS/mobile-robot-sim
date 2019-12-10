

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>

using namespace arpro;
using namespace std;

Environment *Sensor::envir_ = nullptr;

Robot::Robot(std::string _name, double _x, double _y, double _theta)
{
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);

    // default sampling time: 1/100 s
    dt_ = .01;
    wheels_init_ = false;
}

void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx * dt_;
    pose_.y += _vy * dt_;
    pose_.theta += _omega * dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}

void Robot::rotateWheels(double _left, double _right)
{
    // to fill up after defining an initWheel method
    if (wheels_init_)
    {

        double a = std::max(fabs(_left) / wmax_, fabs(_right) / wmax_);

        if (a < 1)
        {
            a = 1;
        }

        double left = _left / a;
        double right = _right / a;

        double nv = r_ * (left + right) / 2;
        double nw = r_ * (left - right) / (2 * b_);

        moveXYT(nv * cos(pose_.theta), nv * sin(pose_.theta), nw);
    }
}

// move robot with linear and angular velocities
void Robot::moveVW(double v, double w)
{
    /*double vx = v * cos(pose_.theta);
    double vy = v * sin(pose_.theta);

    moveXYT(vx, vy, w);*/

    double wl = (v + b_ * w) / r_;
    double wr = (v - b_ * w) / r_;
    rotateWheels(wl, wr);
}

// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}

void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist correcting

    for (int i = 0; i < sensors_.size(); i++)
    {
        sensors_[i]->updateFromRobotPose(pose());
        sensors_[i]->correctRobotTwist(_twist);
    }

    // uses X-Y motion (perfect but impossible in practice)
    // moveXYT(_twist.vx, _twist.vy,_twist.w);

    // to fill up, use V-W motion when defined
    moveVW(_twist.vx, 20 * _twist.vy + _twist.w);
}

void Robot::initWheels(double b, double r, double wmax)
{
    b_ = b;
    r_ = r;
    wmax_ = wmax;
    wheels_init_ = true;
}

void Robot::printPosition()
{
    std::cout << "Current position: " << pose_.x << ", " << pose_.y << std::endl;
}
