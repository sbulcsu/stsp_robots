#include <assert.h>
#include <selforg/matrix.h>
#include <osg/Matrix>
#include <ode_robots/irsensor.h>
#include <ode_robots/osgprimitive.h> // get access to graphical (OSG) primitives
#include <ode_robots/mathutils.h>

#include "barrel_robot.h"

using namespace osg;
using namespace std;

namespace lpzrobots {


void BarrelRobotConf::destroy(){
     for(list<Sensor*>::iterator i = sensors.begin(); i != sensors.end(); i++){
       if(*i) delete *i;
     }
     sensors.clear();
};

//const int BarrelRobot::servono;

BarrelRobot::BarrelRobot ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                               const BarrelRobotConf& conf,const std::string& name,
                               double transparency, int axes_number)
    : OdeRobot( odeHandle, osgHandle, name, "$Id$"),  conf(conf), transparency(transparency){
       numberaxis= axes_number;
       init();
};


BarrelRobot::~BarrelRobot(){ 
      destroy();
      if(conf.irSensorTempl) delete conf.irSensorTempl; // ersetzen durch BarrelRobotConf::destroy???
      FOREACH(std::list<Sensor*>, conf.sensors, s){     // c.destroy() 
        if(*s) delete *s;
      }
};


void BarrelRobot::update(){    //   update of primitives and osgprimitives
     for (int i=0; i < Last; i++) {
       if(objects[i]) objects[i]->update();
     }
     Matrix pose(objects[Base]->getPose());
     for (int i=0; i < servono; i++) {
       if(axis[i]){
         axis[i]->setMatrix(Matrix::rotate(M_PI/2, (i==1), (i==0), (i==2)) * 
                            Matrix::translate(0 ,0, (i==0?-1:1)*conf.axesShift)* pose);
	 axisdots[i]->setMatrix(Matrix::rotate(M_PI/2, (i==1), (i==0), (i==2)) * 
                            Matrix::translate(i==0?conf.diameter/2:0, i==1?conf.diameter/2:0 , 0)* pose);
       }
     }
     irSensorBank.update();
     FOREACH(std::list<Sensor*>, conf.sensors, s){
       (*s)->update();
     }
};


/**
 *Writes the sensor values to an array in the memory.
 *@param sensor* pointer to the array
 *@param sensornumber length of the sensor array
 *@return number of actually written sensors
 **/
int BarrelRobot::getSensorsIntern( sensor* sensors, int sensornumber ){
     int len=0;
     assert(created);
     if(!conf.motor_ir_before_sensors){
       FOREACH(list<Sensor*>, conf.sensors, i){
         len += (*i)->get(sensors+len, sensornumber-len);
       }
     }
     if(conf.motorsensor){
       for ( unsigned int n = 0; n < numberaxis; n++ ) {
         sensors[len] = servo[n]->get(); 
         len++;
       }
     }
     // reading ir sensorvalues
     if (conf.irAxis1 || conf.irAxis2 || conf.irAxis3){
       len += irSensorBank.get(sensors+len, sensornumber-len);
     }
    if(conf.motor_ir_before_sensors){
       FOREACH(list<Sensor*>, conf.sensors, i){
         len += (*i)->get(sensors+len, sensornumber-len);
       }
     }
     return len;
};



void BarrelRobot::setMotorsIntern( const double* motors, int motornumber ) {
  int len = min((unsigned)motornumber, numberaxis);
  for ( int n = 0; n < len; n++ ) {
    servo[n]->set(motors[n]);
  }
};


//**************** wird jedes mal aufgerufen
//cout << " getMotorNumberIntern wurde aufgerufen" << endl;
int BarrelRobot::getMotorNumberIntern(){
  return numberaxis;
};

//**************** wird fast nie aufgerufen
//cout << "             getSensorNumberIntern wurde aufgerufen" << endl;
int BarrelRobot::getSensorNumberIntern() {
  int s=0;
  FOREACHC(list<Sensor*>, conf.sensors, i){
    s += (*i)->getSensorNumber();
  }
  return conf.motorsensor * numberaxis + s + irSensorBank.getSensorNumber();
};


void BarrelRobot::placeIntern(const osg::Matrix& pose){
  osg::Matrix p2;
  p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.diameter/2));
  create(p2);
};


void BarrelRobot::sense(GlobalData& globalData) {
  // reset ir sensors to maximum value
  irSensorBank.sense(globalData);
  OdeRobot::sense(globalData);
};


void BarrelRobot::doInternalStuff(GlobalData& global){
     OdeRobot::doInternalStuff(global);
     //// slow down rotation around z axis because friction does not catch it.
     //dBodyID b = getMainPrimitive()->getBody();
     //double friction = odeHandle.substance.roughness;
     //const double* vel = dBodyGetAngularVel( b);
     //if(fabs(vel[2])>0.2){
     //  dBodyAddTorque ( b , 0 , 0 , -0.05*friction*vel[2] );
     //}
     //// deaccelerates the robot
     //if(conf.brake){
     //  dBodyAddTorque ( b , -conf.brake*vel[0] , -conf.brake*vel[1] , -conf.brake*vel[2] );
     //}
     FOREACH(list<Sensor*>, conf.sensors, s){
       (*s)->sense(global);
     }
};


void BarrelRobot::notifyOnChange(const paramkey& key){
     if(key == "motorpower" || key == "pendularmass"){
	for(unsigned int i=0; i<numberaxis; i++){
	    servo[i]->setPower(conf.pendularmass*conf.motorpowerfactor);
	    servo[i]->setDamping(sqrt(4*conf.pendularmass/(conf.pendularmass*conf.motorpowerfactor))); 
	}
	cout << " changed power and damping of pd-controller" << endl;
     }
     if(key == "pendularrange"){
	for(unsigned int i=0; i<numberaxis; i++){
	    servo[i]->setMinMax(-0.5*conf.diameter*conf.pendularrange, 0.5*conf.diameter*conf.pendularrange);
	}
	cout << " changed pendularrange of sliders" << endl;
     }
     if(key == "pendularmass"){
	objects[Pendular1]->setMass(conf.pendularmass);    
	if(objects[Pendular2]) objects[Pendular2]->setMass(conf.pendularmass);    
	cout << " changed pendularmass of sliders" << endl;
     }
};


/** creates vehicle at desired position and orientation */
void BarrelRobot::create(const osg::Matrix& pose){
     if (created){  destroy(); }
   
     // create vehicle space and add it to the top level space
     odeHandle.createNewSimpleSpace(parentspace,true);           //parentspace?????
     Color c(osgHandle.color);
     c.alpha() = transparency;
     OsgHandle osgHandle_Base = osgHandle.changeColor(c);
     OsgHandle osgHandleX[servono];
     osgHandleX[0] = osgHandle.changeColor(Color(1.0, 0.0, 0.0));
     osgHandleX[1] = osgHandle.changeColor(Color(0.0, 1.0, 0.0));
   
     objects[Base] = new Cylinder(conf.diameter/2, conf.diameter);
     objects[Base]->init(odeHandle, conf.spheremass, osgHandle_Base);
     objects[Base]->setPose(pose);
   
     Pos p(pose.getTrans());                                   //pose.getTrans()?????
     Primitive* pendular[servono];
     memset(pendular, 0, sizeof(void*)* servono);

     //definition of the 3 Slider-Joints, which are the controled by the robot-controler
     for ( unsigned int n = 0; n < numberaxis; n++ ) {
       pendular[n] = new Sphere(conf.pendulardiameter/2);
       pendular[n]->init(odeHandle, conf.pendularmass, osgHandleX[n],
                         Primitive::Body | Primitive::Draw);                  
       pendular[n]->setPose(Matrix::translate(0,0,(n==0?-1:1)*conf.axesShift)*pose);
   
       joints[n] = new SliderJoint(objects[Base], pendular[n], p, Axis((n==0), (n==1), (n==2))*pose);
       joints[n]->init(odeHandle, osgHandle, false);
       joints[n]->setParam ( dParamLoStop, -dInfinity);
       joints[n]->setParam ( dParamHiStop, -dInfinity);
       //joints[n]->setParam ( dParamStopCFM, 0.1);  // what
       //joints[n]->setParam ( dParamStopERP, 0.9);  //  do
       //joints[n]->setParam ( dParamCFM, 0.001);    // they influence??


       servo[n] = new SliderServo(dynamic_cast<OneAxisJoint*>(joints[n]), 
                                  -0.5*conf.diameter*conf.pendularrange, 
                                  0.5*conf.diameter*conf.pendularrange, 
                                  conf.pendularmass*conf.motorpowerfactor, 
                                  sqrt(4*conf.pendularmass/(conf.pendularmass*conf.motorpowerfactor)), 
                                  0, 100, dInfinity);
       //servo[n]->setBaseName("Position of ");
       //const std::string name = "Pendular " + itos(n);
       //servo[n]->setNames("Pendular " );

       axis[n] = new OSGCylinder(conf.diameter/100, conf.diameter - conf.diameter/100);
       axis[n]->init(osgHandleX[n], OSGPrimitive::Low);
       axisdots[n] = new OSGSphere(conf.pendulardiameter/3);
       axisdots[n]->init(osgHandleX[n], OSGPrimitive::Middle);
       axisdots[n]->setMatrix(Matrix::translate(n==0?conf.diameter/2: 0, n==1?conf.diameter/2: 0, 0)*pose);
     }
     objects[Pendular1] = pendular[0];
     if(numberaxis==2) objects[Pendular2] = pendular[1];
   
     /** begin with Sensor initialisation **/ 
     double sensorrange = conf.irsensorscale * conf.diameter;
     RaySensor::rayDrawMode drawMode = conf.drawIRs;
     double sensors_inside=0.02;
     if(conf.irSensorTempl==0){
       conf.irSensorTempl=new IRSensor(conf.irCharacter);
     }
     irSensorBank.setInitData(odeHandle, osgHandle, TRANSM(0,0,0) );
     irSensorBank.init(0);
     if (conf.irAxis1){
       for(int i=-1; i<2; i+=2){
         RaySensor* sensor = conf.irSensorTempl->clone();
         Matrix R = Matrix::rotate(i*M_PI/2, 1, 0, 0) *
           Matrix::translate(0,-i*(conf.diameter/2-sensors_inside),0 );
         irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
       }
     }
     if (conf.irAxis2){
       for(int i=-1; i<2; i+=2){
         RaySensor* sensor = conf.irSensorTempl->clone();
         Matrix R = Matrix::rotate(i*M_PI/2, 0, 1, 0) *
           Matrix::translate(i*(conf.diameter/2-sensors_inside),0,0 );
         irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
       }
     }
     if (conf.irAxis3){
       for(int i=-1; i<2; i+=2){
         RaySensor* sensor = conf.irSensorTempl->clone();
         Matrix R = Matrix::rotate( i==1 ? 0 : M_PI, 1, 0, 0) *
           Matrix::translate(0,0,i*(conf.diameter/2-sensors_inside));
         irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
       }
     }
     FOREACH(list<Sensor*>, conf.sensors, i){
       (*i)->init(objects[Base]);
     }
     created=true;
};


void BarrelRobot::init(){
     created = false;
     objects.resize(Last); //hier werden die größen der Listen festgelegt
     joints.resize(2*servono);
     memset(axis, 0, sizeof(void*)* servono);
     memset(servo, 0, sizeof(void*)* servono); //und zwar von: objects, joints, axis, servo
     this->conf.pendulardiameter = conf.diameter/7;
     addParameter("motorpower", &this->conf.motorpowerfactor, 0, 500, "from 0 to 500");
     addParameter("pendularrange",&this->conf.pendularrange,0,1, "from 0 to 1 (bounds of slider joints = robots surface)");
     addParameter("pendularmass",&this->conf.pendularmass,0,0.4, "mass of the slider");
};


void BarrelRobot::destroy(){   /** destroys vehicle and space */
  if (created){
    for (int i=0; i<servono; i++){
      if(servo[i]) delete servo[i];
      if(axis[i]) delete axis[i];
    }
    irSensorBank.clear();
    odeHandle.deleteSpace();
  }
  created=false;
};


};
