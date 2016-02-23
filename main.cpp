#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
//#include <ode_robots/playground.h>
//#include <ode_robots/octaplayground.h>
//#include <ode_robots/terrainground.h>
//#include <ode_robots/passivesphere.h>
//#include <ode_robots/passivebox.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>               // difference betweend these
//#include <selforg/selectiveone2onewiring.h>      // two wirings?
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
  STSPController* controller;  
  OdeAgent* agent;
  OdeRobot* robot;
  double friction;
  bool track = true; 

  enum robotType {TypeBarrel, TypeSphere};
  robotType type= TypeSphere;

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("colors.txt");
    setGroundTexture("Images/whiteground.jpg");\
  }

  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    /*********** WORLDSPARAMETER **********/
    global.odeConfig.setParam("noise",0);
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize", 0.01);
    global.odeConfig.addParameterDef("friction", &friction, 0.3, "rolling friction coefficient");
    
    /********** CAMERA **********/
    setCameraMode(Follow);            //Follow, Race or Static
    //setCameraHomePos(Pos(-0.535584, 13.4922, 6.79505),  Pos(-177.933, -25.1901, 0));
    setCameraHomePos(Pos(0.0303593, 6.97324, 3.69894),  Pos(-177.76, -24.8858, 0));


    /*********** ROBOTS  **********/
    if(type == TypeBarrel){
       BarrelRobotConf conf = BarrelRobot::getDefaultConf();
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
       controller = new STSPController( global.odeConfig );
       One2OneWiring* wiring = new One2OneWiring( new ColorUniformNoise(0.1) );
       agent = new OdeAgent( globalData );
       agent->init( controller, robot, wiring );
       //agent->fixateRobot(global,2,10);
       //FixedJoint* fixedJoint = new FixedJoint(robot->getMainPrimitive(), global.environment);
       //fixedJoint->init(odeHandle, osgHandle, true);
       global.agents.push_back( agent );
       global.configs.push_back( controller );
       global.configs.push_back( robot ); 
    }
    else if(type == TypeSphere){
       SphereRobotConf sconf = SphereRobot::getDefaultConf();
       sconf.diameter = 0.5;
       sconf.motorpowerfactor = 120;

       robot = new SphereRobot( odeHandle, osgHandle.changeColor(Color(0.,0.,1.)), sconf, "Sphere", 0.4);
       robot->addSensor(std::make_shared<SpeedSensor>( 1, SpeedSensor::Translational ),Attachment(-1));
       robot->place(osg::Matrix::translate(0,0,0.3));
       controller = new STSPController( global.odeConfig );
       One2OneWiring* wiring = new One2OneWiring( new ColorUniformNoise(0.1));
       agent = new OdeAgent( globalData );
       agent->init( controller, robot, wiring );
       global.agents.push_back( agent );
       global.configs.push_back( controller );
       global.configs.push_back( robot );
    }
    if(track == true)  agent->setTrackOptions(TrackRobot(false, false, false, true, "ZSens", 50));
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
        case 'j' : dBodyAddTorque( robot->getMainPrimitive()->getBody() , 100 , 0 , 0 ); break;
        case 'J' : dBodyAddTorque( robot->getMainPrimitive()->getBody() , -100 , 0 , 0 ); break;
        case 'k' : dBodyAddTorque( robot->getMainPrimitive()->getBody() , 0 , 100 , 0 ); break;
        case 'K' : dBodyAddTorque( robot->getMainPrimitive()->getBody() , 0 , -100 , 0 ); break;
        case 'l' : dBodyAddTorque( robot->getMainPrimitive()->getBody() , 0, 0, 100 ); break;
        case 'L' : dBodyAddTorque( robot->getMainPrimitive()->getBody() , 0, 0, -100 ); break;
        case 'u' : dBodyAddForce( robot->getMainPrimitive()->getBody() , 100, 0, 0 ); break;
        case 'U' : dBodyAddForce( robot->getMainPrimitive()->getBody() , -100, 0, 0 ); break;
        case 'i' : dBodyAddForce( robot->getMainPrimitive()->getBody() , 0, 100, 0 ); break;
        case 'I' : dBodyAddForce( robot->getMainPrimitive()->getBody() , 0, -100, 0 ); break;
        case 'a' : controller->increaseA(-0.02);
                   std::cout << "new a:  "<<controller->getParam("a") << std::endl;
                   break;
        case 'A' : controller->increaseA(0.02);
                   std::cout << "new a:  "<<controller->getParam("a") << std::endl;
                   break;
        case 'w' : controller->increaseW(-1);
                   std::cout << "new w_0:  "<<controller->getParam("w_0") << std::endl;
                   break;
        case 'W' : controller->increaseW(1);
                   std::cout << "new w_0:  "<<controller->getParam("w_0") << std::endl;
                   break;
        case 'z' : controller->increaseZ(1.);
                   std::cout<< "new z_0:  "<<controller->getParam("z_0")<< std::endl;
                   break;
        case 'Z' : controller->increaseZ(-1.);
                   std::cout<< "new z_0:  "<<controller->getParam("z_0")<< std::endl;
                   break;
        case 'g' : controller->increaseGamma(-0.1);
                   std::cout<< "new gamma:  "<<controller->getParam("gamma")<< std::endl;
                   break;
        case 'G' : controller->increaseGamma(0.1);
                   std::cout<< "new gamma:  "<<controller->getParam("gamma")<< std::endl;
                   break;
	case 'b' : controller->setRandomPhi(); break;
	case 'n' : controller->setRandomU(); break;
	case 'm' : controller->setRandomX(10.); break;
	case 'r' : controller->setRandomAll(10.); break;
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

