#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>               // difference betweend these
#include <selforg/selectiveone2onewiring.h>      // two wirings?
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>
#include <ode_robots/sensor.h>

#include "sphere_robot.h"
#include "barrel_robot.h"
#include "stsp_controller.h"

using namespace lpzrobots;
using namespace std;


class ThisSim : public Simulation {
public:
  AbstractController* controller;  
  OdeRobot* robot;
  Sensor* sensor;
  double friction;
  bool track = false; 

  enum robotType {TypeBarrel, TypeSphere};
  robotType type= TypeSphere;

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("colors.txt");
    setGroundTexture("Images/whiteground.jpg");\
  }

  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    sensor=0;
    /*********** WORLDSPARAMETER **********/
    global.odeConfig.setParam("noise",0);
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize", 0.01);
    global.odeConfig.addParameterDef("friction", &friction, 0.3, "rolling friction coefficient");
    
    /********** Camera **********/
    setCameraMode(Follow);            //Follow, Race or Static
    setCameraHomePos(Pos(-0.535584, 13.4922, 6.79505),  Pos(-177.933, -25.1901, 0));

    if(type == TypeBarrel){
       BarrelRobotConf  conf = BarrelRobot::getDefaultConf();
       conf.diameter = 2.0;
       conf.pendularrange  = .5;  // 1. at the surface 
       conf.spheremass   = 1;
       conf.pendularmass = 1;
       conf.motorpowerfactor  = 30;  
       conf.axesShift = 0;
       conf.motorsensor=true;           
       robot = new BarrelRobot(odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)),
                               conf, "Barrel", 0.4, 2); 
       robot->place (osg::Matrix::rotate(M_PI/2, 1,0,0)*osg::Matrix::translate(0,0,0.3)); 
       robot->addSensor(std::make_shared<SpeedSensor>( 1,SpeedSensor::Translational ),Attachment(-1));
       robot->addSensor(std::make_shared<SpeedSensor>( 1,SpeedSensor::Rotational ),Attachment(-1));
       controller = new STSPController( global.odeConfig, conf.diameter, conf.pendularrange );
       One2OneWiring* wiring = new One2OneWiring( new ColorUniformNoise(0.1) );
       OdeAgent* agent = new OdeAgent( globalData );
       agent->init( controller, robot, wiring );
       if(track == true)  agent->setTrackOptions(TrackRobot(false, false, false, true, "ZSens", 50));
       //agent->fixateRobot(global,2,10);
       //FixedJoint* fixedJoint = new FixedJoint(robot->getMainPrimitive(), global.environment);
       //fixedJoint->init(odeHandle, osgHandle, true);
       global.agents.push_back( agent );
       global.configs.push_back( controller );
       global.configs.push_back( robot ); 
    }

    else if(type == TypeSphere){
       SphereRobotConf sconf = SphereRobot::getDefaultConf();
       robot = new SphereRobot( odeHandle, osgHandle, sconf, "Sphere", 0.4);
       robot->place(osg::Matrix::translate(0,0,0.3));
       controller = new STSPController( global.odeConfig, sconf.diameter, sconf.pendularrange);
       One2OneWiring* wiring = new One2OneWiring( new ColorUniformNoise(0.1));
       OdeAgent* agent = new OdeAgent( globalData );
       agent->init( controller, robot, wiring );
       global.agents.push_back( agent );
       global.configs.push_back( controller );
       global.configs.push_back( robot );
    }
  };




  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(!pause){    // if not really neccesary here..?
       if(friction>0){
          OdeRobot* robot = globalData.agents[0]->getRobot();
          Pos vel = robot->getMainPrimitive()->getAngularVel();
          robot->getMainPrimitive()->applyTorque(-vel*friction);
       }
    }
  }



  
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down){
    if (down) { // only when key is pressed, not when released
      switch ( (char) key )
        {
        case 'x' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 100 , 0 , 0 ); break;
        case 'X' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , -100 , 0 , 0 ); break;
        case 'y' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , 100 , 0 ); break;
        case 'Y' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , -100 , 0 ); break;
        case 'z' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0, 0, 100 ); break;
        case 'Z' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0, 0, -100 ); break;
        case 'q' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , 100, 0, 0 ); break;
        case 'Q' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , -100, 0, 0 ); break;
        case 'w' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , 0, 100, 0 ); break;
        case 'W' : dBodyAddForce ( robot->getMainPrimitive()->getBody() , 0, -100, 0 ); break;
        case 'a' : controller->setParam("a", controller->getParam("a")-0.02);
                   std::cout << "all a changed to:  "<<controller->getParam("a") << std::endl;
                   break;
        case 'A' : controller->setParam("a", controller->getParam("a")+0.02);
                   std::cout << "all a changed to:  "<<controller->getParam("a") << std::endl;
                   break;
        case 's' : controller->setParam("w_0", controller->getParam("w_0")-10);
                   std::cout << "all w_0 changed to:  "<<controller->getParam("w") << std::endl;
                   break;
        case 'S' : controller->setParam("w_0", controller->getParam("w_0")+10);
                   std::cout << "all w_0 changed to:  "<<controller->getParam("w_0") << std::endl;
                   break;
        case 'd' : controller->setParam("z_0", controller->getParam("z_0")-10);
                   std::cout<< "all z_0 changed to:  "<<controller->getParam("z_0")<< std::endl;
                   break;
        case 'D' : controller->setParam("z_0", controller->getParam("z_0")+10);
                   std::cout<< "all z_0 changed to:  "<<controller->getParam("z_0")<< std::endl;
                   break;
        default:
                return false;
                break;
        }
    }
    return false;
  }
};

int main (int argc, char **argv){
  ThisSim sim;
  sim.setCaption("Robot with STSP controller (lpzrobots Simulator)   Martin 2016");
  return sim.run(argc, argv) ? 0 : 1;
}

