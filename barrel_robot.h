#ifndef __SPHEREROBOT3MASSES_H
#define __SPHEREROBOT3MASSES_H

#include <ode_robots/primitive.h>
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/oderobot.h>
#include <ode_robots/sensor.h>
#include <ode_robots/raysensorbank.h>

namespace lpzrobots {


typedef struct {
public:
     double diameter;
     double spheremass;
     double pendulardiameter; ///< automatically set
     double pendularmass;
     double motorpowerfactor; ///< power factor for servos w.r.t. pendularmass
     double pendularrange;    ///< fraction of the diameter the pendular masses can move to one side
     bool motorsensor;        ///< motor values as sensors
     bool irAxis1;
     bool irAxis2;
     bool irAxis3;
     RaySensor::rayDrawMode drawIRs;
     double irsensorscale; ///< range of the ir sensors in units of diameter
     double irCharacter;   ///< characteristics of sensor (\f[ x^c \f] where x is the range-distance)
     RaySensor* irSensorTempl;  ///< template for creation of the other ir sensors (if 0 then IRSensor(irCharacter))
     double motor_ir_before_sensors; ///< if true motor sensors and ir sensors are given before additional sensors
     double brake;         ///< if nonzero the robot brakes (deaccelerates actively/magically)
     double axesShift; ///< defines how much the axes are shifted from the center
   
     /// function that deletes sensors
     void destroy();
     /// list of sensors that are mounted at the robot. (e.g.\ AxisOrientationSensor)
     std::list<Sensor*> sensors;
     /// adds a sensor to the list of sensors
     void addSensor(Sensor* s) { sensors.push_back(s); }
} BarrelRobotConf;


class BarrelRobot : public OdeRobot
{
   public:

     BarrelRobot ( const OdeHandle& odeHandle, const OsgHandle& osgHandle, const BarrelRobotConf& conf,
                     const std::string& name, double transparency=0.5, int axes_number=2 );
   
     virtual ~BarrelRobot();

     static BarrelRobotConf getDefaultConf(){ // default configuration
       BarrelRobotConf c;
       c.diameter     = 2.;
       c.spheremass   = 100.;// 0.1
       c.pendularmass  = 1.0;
       c.pendularrange  = 0.5; // range of the slider from center in multiple of diameter [-range,range]
       c.motorpowerfactor  = 100;
       c.motorsensor = true;
       c.irAxis1=false;
       c.irAxis2=false;
       c.irAxis3=false;
       c.drawIRs=RaySensor::drawAll;
       c.irsensorscale=1.5;
       c.irCharacter=1;
       c.irSensorTempl=0;
       c.motor_ir_before_sensors=true;
       c.brake=0;
       c.axesShift=0;
      return c;
     }

     enum parts { Base, Pendular1, Pendular2, Enddot1, Enddot2, Last };
   
     virtual void update();
   
     virtual int getSensorsIntern( sensor* sensors, int sensornumber );
   
     virtual void setMotorsIntern( const double* motors, int motornumber );
   
     virtual int getMotorNumberIntern();
   
     virtual int getSensorNumberIntern();
   
     virtual void placeIntern(const osg::Matrix& pose);

     virtual void sense(GlobalData& globalData) override;
   
     virtual void doInternalStuff(GlobalData& globalData);
   
     virtual void notifyOnChange(const paramkey& key);
   
   
   protected:
     virtual void create(const osg::Matrix& pose);

     void init();  // initialises some internal variables

     virtual void destroy();

     static const int servono= 2;
     unsigned int numberaxis;
   
     SliderServo* servo[servono];
     OSGPrimitive* axis[servono];
     OSGPrimitive* axisdots[servono];
   
     BarrelRobotConf conf;
     RaySensorBank irSensorBank;   //< a collection of ir sensors
     double transparency;
     bool created;

    // BarrelRobot ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
    //                 const BarrelRobotConf& conf, const std::string& name,
    //                 const std::string& revision, double transparency);
   
};

}

#endif
