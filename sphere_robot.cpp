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
#include <ode_robots/osgprimitive.h>
#include <ode_robots/mathutils.h>

#include "sphere_robot.h"

using namespace osg;
using namespace std;

namespace lpzrobots {
/********************/
  SphereRobot::SphereRobot( const OdeHandle& odeHandle, 
                            const OsgHandle& osgHandle,
                            const SphereRobotConf& conf,
                            const std::string& name,
			    const OdeConfig& odeconfig, 
                            double transparency, unsigned int axes_number)
    : OdeRobot( odeHandle, osgHandle, name,"$Id$"), conf(conf), transparency(transparency),
      numberaxis(axes_number), odeconfig(odeconfig)
  {
       vMean = Pos(0,0,0);
       vMeanTot = 0.;
       vMeanTot2 = 0.;
       stepsize = odeconfig.simStepSize;
       if( conf.ave_speedsensors == true) number_speedsensors = 6;
       else number_speedsensors = 0; 

       created = false;
       objects.resize(1+numberaxis);   //+1 because of base
       joints.resize(numberaxis);
       memset(axis, 0, sizeof(void*) * numberaxis); 
       memset(servo, 0, sizeof(void*) * numberaxis);
       this->conf.pendulardiameter = conf.diameter/7;
   
       addParameter("pendularmass",&this->conf.pendularmass, "mass of the slider");
       addParameter("motorpower", &this->conf.motorpowerfactor, 
		    "K_p parameter of PD controller = motorpower*pendularmass ");
       addParameter("pendularrange",&this->conf.pendularrange,   
		    "range of the masses along the sliders");
       addParameterDef("T_ave",&this->T_ave, 100, "Timescale of the trailing speed average");
  }
/********************/
  SphereRobot::~SphereRobot(){
               destroy();
  }
/********************/
  void SphereRobot::update(){
       for( unsigned int i=0; i < (1+numberaxis); i++) {  //loop over all parts of the robot    
         if(objects[i]) objects[i]->update();
       }
       Matrix pose(objects[Base]->getPose());	//updates the osgparts
       for( unsigned int i=0; i < numberaxis; i++) {
         if(axis[i]){
            axis[i]->setMatrix(Matrix::rotate(M_PI/2, (i==1), (i==0), (i==2)) * 
                               Matrix::translate(0 ,0, (i==0?-1:1)*conf.axesShift)* pose);
            axisdots[i]->setMatrix(Matrix::rotate(M_PI/2, (i==1), (i==0), (i==2)) *
                         Matrix::translate(i==0?conf.diameter/2:0, 
					   i==1?conf.diameter/2:0, 
			                   i==2?conf.diameter/2:0)* pose);
         }
       }
  }
/********************/   // garantees that the robot is placed above the ground
  void SphereRobot::placeIntern(const osg::Matrix& pose){
       osg::Matrix p2;
       p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.diameter/2));
       create(p2);
  }
/********************/   
  int SphereRobot::getSensorsIntern( sensor* sensors, int sensornumber) {
      int len=0;
      assert(created);
      for(unsigned int n = 0; n < numberaxis; n++ ) {
      	  sensors[len] = servo[n]->get();            
          len++;
      }
      for(unsigned int n = 0; n< numberaxis; n++){
          sensors[len] = servo[n]->get()* conf.pendularrange* 0.5*  conf.diameter;
          len++; 
      }
      if(conf.ave_speedsensors == true){
	 stepsize = odeconfig.simStepSize;
	 Pos vel = getSpeed();
	 vMean = trailingAverage( vel, vMean );
         for(int i=0; i<3; i++){
	     sensors[len] = vMean[i];
	     len++;
	 }
         vMeanTot2 += ( sqrt(pow(vel[0],2)+pow(vel[1],2)+pow(vel[2],2)) -vMeanTot2)* stepsize / T_ave;
	 sensors[len] = vMeanTot2;
	 len++;
	 vMeanTot = sqrt( pow(vMean[0],2) + pow(vMean[1],2) + pow(vMean[2],2) );
	 sensors[len] = vMeanTot;
	 len++;
	 sensors[len] = sqrt( pow(vel[0],2) + pow(vel[1],2) + pow(vel[2],2) );  
	 len++;
      }
      return len;
  }
/********************/
  Pos SphereRobot::trailingAverage( Pos vel, Pos vMean_old){
      Pos vMean_new;
      for( int i=0 ; i<3; i++){
	   vMean_new[i] = vMean_old[i] + (vel[i] - vMean_old[i])*stepsize / T_ave;
	 }
      return vMean_new;
  }
/********************/
  void SphereRobot::setMotorsIntern( const double* motors, int motornumber ) {
       int len = min((unsigned)motornumber, numberaxis);
       for ( int n = 0; n < len; n++ ) {
         servo[n]->set(motors[n]);
       }
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
       if(key == "pendularrange"){    
          for(unsigned int i=0; i<numberaxis; i++){
             servo[i]->setMinMax(-0.5*conf.diameter*conf.pendularrange, 
        			  0.5*conf.diameter*conf.pendularrange);
          }
          cout << " changed pendularrange of sliders" << endl;
       }
       if(key == "pendularmass"){
	  for(unsigned int i=0; i < numberaxis; i++){
	      objects[1+i]->setMass(conf.pendularmass); 
	  }
          cout << " changed pendularmass of sliders" << endl;
       }
  }
/********************/
  void SphereRobot::destroy(){
       if(created){
          for( unsigned int i=0; i<numberaxis; i++){
               if( servo[i] ) delete servo[i];
               if( axis[i] ) delete axis[i];
	       if( axisdots[i] ) delete axisdots[i];
         }
         odeHandle.deleteSpace();
       }
       created=false;
  }
/********************/   
  void SphereRobot::create(const osg::Matrix& pose){
    if(created) destroy();
    odeHandle.createNewSimpleSpace(parentspace,true);
    Color c(osgHandle.color);
    c.alpha() = transparency;
    OsgHandle osgHandle_Base = osgHandle.changeColor(c);
    OsgHandle osgHandleX[servono]; 
    osgHandleX[0] = osgHandle.changeColor(Color(1.0, 0.0, 0.0));
    osgHandleX[1] = osgHandle.changeColor(Color(0.0, 1.0, 0.0));
    osgHandleX[2] = osgHandle.changeColor(Color(0.0, 0.0, 1.0));
    objects[Base] = new Sphere(conf.diameter/2);
    objects[Base]->init(odeHandle, conf.spheremass, osgHandle_Base);
    objects[Base]->setPose(pose);
    Pos p(pose.getTrans());
    Primitive* pendular[servono];    
    memset(pendular, 0, sizeof(void*) * servono);
    for( unsigned int n = 0; n < numberaxis; n++ ) {
         pendular[n] = new Sphere(conf.pendulardiameter/2);
         pendular[n]->init(odeHandle, conf.pendularmass, osgHandleX[n],
                           Primitive::Body | Primitive::Draw); // without Geom
         pendular[n]->setPose(Matrix::translate(0,0,(n==0?-1:1)*conf.axesShift)*pose);
         joints[n] = new SliderJoint(objects[Base], pendular[n],
                                    p, Axis((n==0), (n==1), (n==2))*pose);
         joints[n]->init(odeHandle, osgHandle, false);
	 //the following 3 parameters are for the potential 
	 //to keep the pendulars dynamics within certain limits, not used since 'dInfinity'
         //joints[n]->setParam ( dParamStopCFM, 0.1);
         //joints[n]->setParam ( dParamStopERP, 0.9);
         //joints[n]->setParam ( dParamCFM, 0.001);
         servo[n] = new SliderServo(dynamic_cast<OneAxisJoint*>(joints[n]),
				    -0.5*conf.diameter*conf.pendularrange, //_min
				    0.5*conf.diameter*conf.pendularrange, //_max
				    conf.pendularmass*conf.motorpowerfactor, //K_p
				    sqrt(4/conf.motorpowerfactor), //K_d
				    0, 100, dInfinity); //K_i, maxVel, maxLim?
         axis[n] = new OSGCylinder( conf.diameter/100, conf.diameter-conf.diameter/100);
         axis[n]->init( osgHandleX[n], OSGPrimitive::Low );
         axisdots[n] = new OSGSphere( conf.pendulardiameter/3 );
         axisdots[n]->init( osgHandleX[n], OSGPrimitive::Middle );
         axisdots[n]->setMatrix( Matrix::translate( n==0?conf.diameter/2: 0, 
						    n==1?conf.diameter/2: 0, 
						    n==2?conf.diameter/2: 0)*pose);
         objects[n+1] = pendular[n]; // (+1) because Base is object[0]
    }
    created=true;
  }




}
