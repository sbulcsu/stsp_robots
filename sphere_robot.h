/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#ifndef __SPHEREROBOT_H
#define __SPHEREROBOT_H

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
    double pendulardiameter;
    double pendularmass;
    double motorpowerfactor; 
    double pendularrange;  
    bool motorsensor;     
    double axesShift; 

    void destroy();
    std::list<Sensor*> sensors;
    void addSensor(Sensor* s) { sensors.push_back(s); }

//IRS    bool irAxis1;            //TODO all ir staff 
//IRS    bool irAxis2;
//IRS    bool irAxis3;
//IRS    bool irRing;         
//IRS    bool irSide;        
//IRS    RaySensor::rayDrawMode drawIRs;
//IRS    double irsensorscale; 
//IRS    double irCharacter;  
//IRS    RaySensor* irSensorTempl;  
//IRS    double motor_ir_before_sensors; 

    double brake;       

} SphereRobotConf;



class SphereRobot : public OdeRobot{
public:
   static SphereRobotConf getDefaultConf(){
          SphereRobotConf c;
          c.diameter    	= 1;
          c.spheremass 		= .3;
          c.pendularmass 	= 1.0;
          c.pendularrange	= 0.20; 
          c.motorpowerfactor 	= 100;
          c.motorsensor		= true;
          c.axesShift		= 0;

//IRS          c.irAxis1		= false;
//IRS          c.irAxis2		= false;
//IRS          c.irAxis3		= false;
//IRS          c.irRing		= false;
//IRS          c.irSide		= false;
//IRS          c.drawIRs=RaySensor::drawAll;
//IRS          c.irsensorscale	= 1.5;
//IRS          c.irCharacter		= 1;
//IRS          c.irSensorTempl	= 0;
//IRS          c.motor_ir_before_sensors = false;

          c.brake		= 0;
          return c;
  } 
  SphereRobot ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                const SphereRobotConf& conf, const std::string& name, 
		double transparency=0.5, int axes_number=3);
  virtual ~SphereRobot();
  virtual void update();
  virtual void placeIntern(const osg::Matrix& pose);
  virtual void doInternalStuff(GlobalData& globalData);
  virtual void sense(GlobalData& globalData) override;
  virtual int getSensorsIntern( sensor* sensors, int sensornumber );
  virtual void setMotorsIntern( const double* motors, int motornumber );
  virtual int getMotorNumberIntern();
  virtual int getSensorNumberIntern();
  virtual void notifyOnChange(const paramkey& key);
  enum parts { Base, Pendular1, Pendular2, Pendular3, Last } ;


protected:
  static const int servono=3;
  virtual void create(const osg::Matrix& pose);
  virtual void destroy();
  void init();
  SliderServo* servo[servono];
  OSGPrimitive* axis[servono];
  OSGPrimitive* axisdots[servono];

  SphereRobotConf conf;
  double transparency;
  unsigned int numberaxis;
  bool created;

  RaySensorBank irSensorBank; 

};

}

#endif
