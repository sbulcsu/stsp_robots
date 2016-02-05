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

#include <assert.h>
#include <selforg/matrix.h>
#include <osg/Matrix>
//IRS#include <ode_robots/irsensor.h>
#include <ode_robots/osgprimitive.h>// get access to graphical (OSG) primitives
#include <ode_robots/mathutils.h>

#include "sphere_robot.h"

using namespace osg;
using namespace std;

namespace lpzrobots {
/********************/
  void SphereRobotConf::destroy(){
    for(list<Sensor*>::iterator i = sensors.begin(); i != sensors.end(); i++){
      if(*i) delete *i;
    }
    sensors.clear();
  }
/********************/
  SphereRobot::SphereRobot ( const OdeHandle& odeHandle, 
                             const OsgHandle& osgHandle,
                             const SphereRobotConf& conf,
                             const std::string& name,
                             double transparency, int axes_number)
    : OdeRobot( odeHandle, osgHandle, name,"$Id$"), conf(conf), transparency(transparency),
      numberaxis(axes_number)
  {
    init();
  }
/********************/
  SphereRobot::~SphereRobot(){
                destroy();
//IRS               if(conf.irSensorTempl) delete conf.irSensorTempl;//TODO
                FOREACH(std::list<Sensor*>, conf.sensors, s){
                  if(*s) delete *s;
                }
  }
/********************/
  void SphereRobot::update(){
       for (int i=0; i < Last; i++) {		//loop over all parts of the robot
         if(objects[i]) objects[i]->update();
       }
       Matrix pose(objects[Base]->getPose());	//updates the osgparts
       for (int i=0; i < servono; i++) {
         if(axis[i]){
            axis[i]->setMatrix(Matrix::rotate(M_PI/2, (i==1), (i==0), (i==2)) * 
                               Matrix::translate(0 ,0, (i==0?-1:1)*conf.axesShift)* pose);
            axisdots[i]->setMatrix(Matrix::rotate(M_PI/2, (i==1), (i==0), (i==2)) *
                         Matrix::translate(i==0?conf.diameter/2:0, 
					   i==1?conf.diameter/2:0, 
			                   i==2?conf.diameter/2:0)* pose);
         }
       }
//IRS       irSensorBank.update();  //TODO
       FOREACH(std::list<Sensor*>, conf.sensors, s){
         (*s)->update();
       }
  }
/********************/   // garantees that the robot is placed above the ground
  void SphereRobot::placeIntern(const osg::Matrix& pose){
       osg::Matrix p2;
       p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.diameter/2));
       create(p2);
  };
/********************/   
  void SphereRobot::doInternalStuff(GlobalData& global){
       OdeRobot::doInternalStuff(global);
       FOREACH(list<Sensor*>, conf.sensors, s){ //TODO
         (*s)->sense(global);
       }
  }
/********************/
  void SphereRobot::sense(GlobalData& globalData) {
       // reset ir sensors to maximum value
//IRS       irSensorBank.sense(globalData);	//TODO
       OdeRobot::sense(globalData);
  }
/********************/
  int SphereRobot::getSensorsIntern( sensor* sensors, int sensornumber) {
      int len=0;
      assert(created);
//IRS      if(!conf.motor_ir_before_sensors){ //TODO
//IRS        FOREACH(list<Sensor*>, conf.sensors, i){
//IRS          len += (*i)->get(sensors+len, sensornumber-len);
//IRS        }
//IRS      }
      if(conf.motorsensor){
        for ( unsigned int n = 0; n < numberaxis; n++ ) {
          sensors[len] = servo[n]->get(); 
          len++;
        }
      }
      // reading ir sensorvalues
//IRS      if (conf.irAxis1 || conf.irAxis2 || conf.irAxis3 || conf.irRing || conf.irSide){ //TODO
//IRS        len += irSensorBank.get(sensors+len, sensornumber-len);
//IRS      }
//IRS     if(conf.motor_ir_before_sensors){
        FOREACH(list<Sensor*>, conf.sensors, i){
          len += (*i)->get(sensors+len, sensornumber-len);
        }
//IRS     }
      return len;
  }
/********************/
  void SphereRobot::setMotorsIntern( const double* motors, int motornumber ) {
       int len = min((unsigned)motornumber, numberaxis);
       for ( int n = 0; n < len; n++ ) {
         servo[n]->set(motors[n]);
       }
  }
/********************/
  int SphereRobot::getMotorNumberIntern(){
      return numberaxis;
  }
/********************/
  int SphereRobot::getSensorNumberIntern() {
      int s=0;
      FOREACHC(list<Sensor*>, conf.sensors, i){
        s += (*i)->getSensorNumber();
      }
//IRS     return conf.motorsensor * numberaxis + s + irSensorBank.getSensorNumber();
/*IRS*/     return conf.motorsensor * numberaxis + s;
  }
/********************/
  void SphereRobot::notifyOnChange(const paramkey& key){
       if(key == "motorpower" || key == "pendularmass"){
          for(unsigned int i=0; i<numberaxis; i++){
              servo[i]->setPower(conf.pendularmass*conf.motorpowerfactor);
              servo[i]->setDamping(sqrt(4*conf.pendularmass/(conf.pendularmass*conf.motorpowerfactor)));
          }
          cout << " changed power and damping of pd-controller" << endl;
       }
       //if(key == "pendularrange"){
       //   for(unsigned int i=0; i<numberaxis; i++){
       //      servo[i]->setMinMax(-conf.diameter*conf.pendularrange, conf.diameter*conf.pendularrange);
       //   }
       //   cout << " changed pendularrange of sliders" << endl;
       //}
       if(key == "pendularmass"){
          objects[Pendular1]->setMass(conf.pendularmass);
          if(objects[Pendular2]) objects[Pendular2]->setMass(conf.pendularmass);
          if(objects[Pendular3]) objects[Pendular3]->setMass(conf.pendularmass);
          cout << " changed pendularmass of sliders" << endl;
       }
  }
/********************/
  void SphereRobot::init(){
       created = false;
       objects.resize(Last);
       joints.resize(servono);
       memset(axis, 0,sizeof(void*) * servono);
       memset(servo, 0,sizeof(void*) * servono);
       this->conf.pendulardiameter = conf.diameter/7;
   
       addParameter("pendularmass",&this->conf.pendularmass,0,0.4, "mass of the slider");
       addParameter("motorpower", &this->conf.motorpowerfactor, 0, 500, "  ");
       //addParameter("pendularrange",&this->conf.pendularrange,0,0.4,"range of the masses along the sliders");
  }
/********************/
  void SphereRobot::destroy(){
       if(created){
         for(int i=0; i<servono; i++){
           if(servo[i]) delete servo[i];
           if(axis[i]) delete axis[i];
         }
//IRS         irSensorBank.clear();
         odeHandle.deleteSpace();
       }
       created=false;
  }
/********************/



  const int SphereRobot::servono;   //TODO




/********************/   //TODO
  void SphereRobot::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    odeHandle.createNewSimpleSpace(parentspace,true);
    Color c(osgHandle.color);
    c.alpha() = transparency;
    OsgHandle osgHandle_Base = osgHandle.changeColor(c);
    OsgHandle osgHandleX[3];
    osgHandleX[0] = osgHandle.changeColor(Color(1.0, 0.0, 0.0));
    osgHandleX[1] = osgHandle.changeColor(Color(0.0, 1.0, 0.0));
    osgHandleX[2] = osgHandle.changeColor(Color(0.0, 0.0, 1.0));

    //objects[Base] = new InvisibleSphere(conf.diameter/2);
    objects[Base] = new Sphere(conf.diameter/2);
    //objects[Base] = new Box(conf.diameter, conf.diameter, conf.diameter);
    objects[Base]->init(odeHandle, conf.spheremass, osgHandle_Base);
    objects[Base]->setPose(pose);

    Pos p(pose.getTrans());
    Primitive* pendular[servono];
    memset(pendular, 0, sizeof(void*) * servono);

    //definition of the 3 Slider-Joints, which are the controled by the robot-controler
    for ( unsigned int n = 0; n < numberaxis; n++ ) {
      pendular[n] = new Sphere(conf.pendulardiameter/2);
      pendular[n]->init(odeHandle, conf.pendularmass, osgHandleX[n],
                        Primitive::Body | Primitive::Draw); // without geom
      pendular[n]->setPose(Matrix::translate(0,0,(n==0?-1:1)*conf.axesShift)*pose);

      joints[n] = new SliderJoint(objects[Base], pendular[n],
                                 p, Axis((n==0), (n==1), (n==2))*pose);
      joints[n]->init(odeHandle, osgHandle, false);
      // the Stop parameters are messured from the initial position!
      // the stops are set by the servo
      // joints[n]->setParam ( dParamLoStop, -1.1*conf.diameter*conf.pendularrange );
      // joints[n]->setParam ( dParamHiStop, 1.1*conf.diameter*conf.pendularrange );

      joints[n]->setParam ( dParamStopCFM, 0.1);
      joints[n]->setParam ( dParamStopERP, 0.9);
      joints[n]->setParam ( dParamCFM, 0.001);
      // see also setParam() for the stops
      servo[n] = new SliderServo(dynamic_cast<OneAxisJoint*>(joints[n]),
                                 -0.5*conf.diameter*conf.pendularrange,
                                 0.5*conf.diameter*conf.pendularrange,
                                 conf.pendularmass*conf.motorpowerfactor,
				 sqrt(4/conf.motorpowerfactor),
				 0, 100, dInfinity);

      axis[n] = new OSGCylinder(conf.diameter/100, conf.diameter - conf.diameter/100);
      axis[n]->init(osgHandleX[n], OSGPrimitive::Low);
      axisdots[n] = new OSGSphere(conf.pendulardiameter/3);
      axisdots[n]->init(osgHandleX[n], OSGPrimitive::Middle);
      axisdots[n]->setMatrix(Matrix::translate(n==0?conf.diameter/2: 0, 
					       n==1?conf.diameter/2: 0, 
					       n==2?conf.diameter/2: 0)*pose);
    }
    objects[Pendular1] = pendular[0];
    objects[Pendular2] = pendular[1];
    objects[Pendular3] = pendular[2];

//IRS    double sensorrange = conf.irsensorscale * conf.diameter;
//IRS    RaySensor::rayDrawMode drawMode = conf.drawIRs;
//IRS    double sensors_inside=0.02;
//IRS    if(conf.irSensorTempl==0){
//IRS      conf.irSensorTempl=new IRSensor(conf.irCharacter);
//IRS    }
//IRS    irSensorBank.setInitData(odeHandle, osgHandle, TRANSM(0,0,0) );
//IRS    irSensorBank.init(0);
//IRS    if (conf.irAxis1){
//IRS      for(int i=-1; i<2; i+=2){
//IRS        RaySensor* sensor = conf.irSensorTempl->clone();
//IRS        Matrix R = Matrix::rotate(i*M_PI/2, 1, 0, 0) *
//IRS          Matrix::translate(0,-i*(conf.diameter/2-sensors_inside),0 );
//IRS        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
//IRS      }
//IRS    }
//IRS    if (conf.irAxis2){
//IRS      for(int i=-1; i<2; i+=2){
//IRS        RaySensor* sensor = conf.irSensorTempl->clone();
//IRS        Matrix R = Matrix::rotate(i*M_PI/2, 0, 1, 0) *
//IRS          Matrix::translate(i*(conf.diameter/2-sensors_inside),0,0 );
//IRS        //        dRFromEulerAngles(R,i*M_PI/2,-i*M_PI/2,0);
//IRS        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
//IRS      }
//IRS    }
//IRS    if (conf.irAxis3){
//IRS      for(int i=-1; i<2; i+=2){
//IRS        RaySensor* sensor = conf.irSensorTempl->clone();
//IRS        Matrix R = Matrix::rotate( i==1 ? 0 : M_PI, 1, 0, 0) *
//IRS          Matrix::translate(0,0,i*(conf.diameter/2-sensors_inside));
//IRS        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
//IRS      }
//IRS    }
//IRS    if (conf.irRing){
//IRS      for(double i=0; i<2*M_PI; i+=M_PI/6){  // 12 sensors
//IRS        RaySensor* sensor = conf.irSensorTempl->clone();
//IRS        Matrix R = Matrix::translate(0,0,conf.diameter/2-sensors_inside) *
//IRS          Matrix::rotate( i, 0, 1, 0);
//IRS        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
//IRS      }
//IRS    }
//IRS    if (conf.irSide){
//IRS      for(double i=0; i<2*M_PI; i+=M_PI/2){
//IRS        RaySensor* sensor = conf.irSensorTempl->clone();
//IRS        Matrix R = Matrix::translate(0,0,conf.diameter/2-sensors_inside) *
//IRS          Matrix::rotate( M_PI/2-M_PI/8, 1, 0, 0) *  Matrix::rotate( i, 0, 1, 0);
//IRS        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
//IRS        sensor = new IRSensor(conf.irCharacter);// and the other side
//IRS        irSensorBank.registerSensor(sensor, objects[Base],
//IRS                                    R * Matrix::rotate( M_PI, 0, 0, 1),
//IRS                                    sensorrange, drawMode);
//IRS      }
//IRS    }

    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->init(objects[Base]);
    }

  created=true;
  }




}
