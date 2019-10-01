#ifndef RANGE_SENSOR_H
#define RANGE_SENSOR_H
#include <math.h>
#include <sensor.h>
namespace arpro
{
class SensorBearing : public Sensor
{
public :
SensorBearing ( Robot & _robot , double _x , double _y , double _theta ) :
Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
{ // the RangeSensor constructor does nothing more }
}


virtual void update(const Pose &_p){

    for ( auto other : envir_->robots_ )
        if ( other != robot_ )
        {
        // compute angle between sensor and detected robot
        s_=atan2(envir_->robots_[0]->pose().y-_p.y,envir_->robots_[0]->pose().x-_p.x)-_p.theta;

        break ;
        }
}






virtual void checkTwist(Twist &_t) {
_t.w=_t.w-0.5*s_;
}};
}

#endif // RANGE_SENSOR_H
