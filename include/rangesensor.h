#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include <sensor.h>
namespace arpro
{
class RangeSensor : public Sensor
{
public:
    RangeSensor(Robot &_robot, double _x, double _y, double _theta) : Sensor(_robot, _x, _y, _theta) // call the Sensor constructor
    {                                                                                                // the RangeSensor constructor does nothing more
    }

    virtual void correctTwist(Twist &_t)
    {
        if (_t.vx > 0.1 * (s_ - 0.1))
        {
            _t.vx = 0.1 * (s_ - 0.1);
        }
    }

    virtual void update(const Pose &_p)
    {
        Pose p1, p2;
        double d[4];
        for (int i = 0; i < envir_->walls.size(); i++)
        {
            p1 = envir_->walls[i];
            p2 = envir_->walls[(i + 1) % envir_->walls.size()];
            // do whatever you want to do with points p1 and p2
            double num = p1.x*p2.y - p1.x*_p.y - p2.x*p1.y + p2.x*_p.y + _p.x*p1.y - _p.x*p2.y;
            double den = p1.x*sin(_p.theta) - p2.x*sin(_p.theta) - p1.y*cos(_p.theta) + p2.y*cos(_p.theta);
            if (den == 0)
            {
                d[i] = 20;
            } else {
                if (num * den < 0)
                {
                    d[i] = 20;
                } else {
                    d[i] = num / den;
                }
            }
        }
        double min = d[0];
        for (int j = 0; j < 4; j++)
        {
            if (d[j] < min)
            {
                min = d[j];
            }
        }

        s_ = min;

        cout << s_ << endl;
    }
};
} // namespace arpro

#endif // RANGE_SENSOR_H
