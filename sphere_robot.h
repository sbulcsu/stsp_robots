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


namespace lpzrobots {

typedef struct {
public:
    double diameter;
    double spheremass;
    double pendulardiameter; //initialized in constructor
    double pendularmass;
    double motorpowerfactor; 
    double pendularrange;  
    double axesShift; 
    //void destroy();
    //std::list<Sensor*> sensors;                          //CONFSENSOR
    //void addSensor(Sensor* s) { sensors.push_back(s); }  //CONFSENSOR
} SphereRobotConf;



class SphereRobot : public OdeRobot{
public:
   static SphereRobotConf getDefaultConf(){
          SphereRobotConf c;
          c.diameter    	= 2.;
          c.spheremass 		= 1.;
          c.pendularmass 	= 1.0;
          c.pendularrange	= 0.5; 
          c.motorpowerfactor 	= 100;
          c.axesShift		= 0;
          return c;
  } 
  SphereRobot ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                const SphereRobotConf& conf, const std::string& name, 
		double transparency=0.5, unsigned int axes_number=3);
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
  enum parts { Base, Pendular1, Pendular2, Pendular3, Last };

protected:
  virtual void create(const osg::Matrix& pose);
  virtual void destroy();
  static const int servono=3;
  SliderServo* servo[servono];
  OSGPrimitive* axis[servono];
  OSGPrimitive* axisdots[servono];

  SphereRobotConf conf;
  double transparency;
  unsigned int numberaxis;
  bool created;


};

}

#endif
