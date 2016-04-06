#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/meshground.h>
//#include <ode_robots/passivesphere.h>
//#include <ode_robots/passivebox.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>               // difference betweend these
//#include <selforg/selectiveone2onewiring.h>      // two wirings?
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>
#include <ode_robots/sensor.h>
#include <ode_robots/primitive.h>

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
  Pos RobInitPos;

  enum robotType {TypeBarrel, TypeSphere};
  robotType type= TypeSphere;
  
  /** Environments: NO: flat ground
  //	if the last argument in the constructor is false, no additional ground is added
  //		    PG: playground fenced by a wall
  //		    OP: polygonal playground fenced by a wall
  //  	TerrainGround objects, their ground is created by using a .ppm file
  //		    HG: flat ground created by white .ppm file
  //		    TB: three basins 
  //		    PT: three potentials different depth
  //		    EL: round or elliptical potential
  //		    ELF: same but different shape
  //		    PLA: plateau
  //		    TI: trench
  //		    RU: 3 little trenchs  */
  enum Env { NO, PG, OP, HG, TB, PT, EL, ELF, PLA, TI, RU};
  Env env = NO;

  ThisSim(){ //for definitions see osg/base.h
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    addColorAliasFile("colors.txt");
    setGroundTexture("Images/whiteground.jpg");
  }

  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    /*********** WORLDSPARAMETER **********/
    global.odeConfig.setParam("noise",0);
    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("realtimefactor",1);
    global.odeConfig.setParam("simstepsize", 0.001);
    global.odeConfig.addParameterDef("friction", &friction, 0.3, "rolling friction coefficient");
    //The default values of substance is defined in odeHandle.substance
    //odeHandle Substance: roughness:  0.8
    //			   slip        0.01
    //			   hardness:   40
    //			   elasticity: 0.5
    //setGroundSubstance( Substance::getPlastic(0.8) );
    Substance GroundSub = getGroundSubstance(); 
    std::cout << "GroundSubstance:	 roughness:  " << GroundSub.roughness << std::endl;
    std::cout << "			 slip:	     " << GroundSub.slip << std::endl;
    std::cout << "			 hardness:   " << GroundSub.hardness << std::endl;
    std::cout << "			 elasticity: " << GroundSub.elasticity << std::endl;
 

    /********** ENVIRONMENT **********/
    createEnv( odeHandle, osgHandle, global, env );

    /** create a plane for testing purposes */
    //auto* plane = new Plane();
    //plane->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
    //plane->setSubstance(Substance::getPlastic(0.8));
    //Substance planeSub = plane->substance; 
    //std::cout << "Plane Substance:       roughness:  " << planeSub.roughness << std::endl;
    //std::cout << "			 slip:	     " << planeSub.slip << std::endl;
    //std::cout << "			 hardness:   " << planeSub.hardness << std::endl;
    //std::cout << "			 elasticity: " << planeSub.elasticity << std::endl;

    //auto* plane1 = new Plane();
    //plane1->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
    //plane1->setSubstance(Substance::getPlastic(0.8));
    //plane1->setSubstance(Substance::getMetal(0.8));
 
    /** create a box for testing purposes */
    //auto* box = new Box(30,30,5);
    //box->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
    //box->setSubstance(Substance::getPlastic(0.8));
    //box->setPosition(Pos(0,0,10)); //member function of Box class
    //Substance boxSub = plane->substance; 
    //std::cout << "Box Substance:         roughness:  " << boxSub.roughness << std::endl;
    //std::cout << "			 slip:	     " << boxSub.slip << std::endl;
    //std::cout << "			 hardness:   " << boxSub.hardness << std::endl;
    //std::cout << "			 elasticity: " << boxSub.elasticity << std::endl;
   

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
       robot->place (osg::Matrix::rotate(M_PI/2, 1,0,0)*osg::Matrix::translate(0,0,10)); 
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
       // to change the substance/material of the robot. be careful: material influences the behaviour
       OdeHandle myHandle = odeHandle;    // default: plastic with roughness= 0.8
       //myHandle.substance.toMetal(0.5);   // roughness [0.1,1], very hard, elastic, slip 
       //myHandle.substance.toRubber(50);   // hardness [5,50], high roughness, no slip, very elastic
       robot = new SphereRobot( myHandle, osgHandle.changeColor(Color(0.,0.,1.)), sconf, 
				"Sphere", global.odeConfig, 0.4);
       robot->addSensor(std::make_shared<SpeedSensor>( 1, SpeedSensor::Translational ),Attachment(-1));
       //robot->place(osg::Matrix::translate(0,0,20));
       robot->place(RobInitPos);
       controller = new STSPController( global.odeConfig );
       One2OneWiring* wiring = new One2OneWiring( new ColorUniformNoise(0.1));
       agent = new OdeAgent( globalData );
       agent->init( controller, robot, wiring );
       global.agents.push_back( agent );
       global.configs.push_back( controller );
       global.configs.push_back( robot );
    }
    /** tracking: ( trackPos, trackSpeed, trackOrientation, displayTrace, scene  = "char", interval ) */
    // if 1 of the first 3 arguments  == true:  log file with values is created 
    TrackRobot* TrackOpt = new TrackRobot(false, false, false, true); 
    TrackOpt->conf.displayTraceDur = 800; //length of track line
    if(track == true)  agent->setTrackOptions( *TrackOpt );
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



  void createEnv( const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
		  GlobalData& global, Env environment ){
    switch(environment){
    case NO: {
	setCameraMode( Follow );        
	setCameraHomePos( Pos(0.0303593, 6.97324, 3.69894),  Pos(-177.76, -24.8858, 0) );
	RobInitPos = Pos(0,0,0);
	} break;
    case PG: {  //Pos( radius, width of wall, hight of wall), bool: has its own ground or not
	// creates two playgrounds next to each other, one using the standard ground (same as 
	// for env = NO), the other uses its own ground which is a box-primitive
	
	//Substance groundSubst = Substance::getSnow(0.1);
	//Substance ground = Substance::getPlastic(0.8);
	

	/** Playground using the normal ground which is of type Plane */
    	//Playground* world1 = new Playground( odeHandle, osgHandle, osg::Vec3(20, 0.2, 0.2), 1, false);
	//setGroundSubstance( groundSubst );
	//setGroundSubstance( ground );
	//Substance GroundSub = getGroundSubstance(); 
	//std::cout << "GroundSubstance:	 roughness:  " << GroundSub.roughness << std::endl;
	//std::cout << "			 slip:	     " << GroundSub.slip << std::endl;
	//std::cout << "			 hardness:   " << GroundSub.hardness << std::endl;
	//std::cout << "			 elasticity: " << GroundSub.elasticity << std::endl;
	//world1->setPosition( osg::Vec3(0,0,0) );
	//global.obstacles.push_back( world1 );

	/** Playground using a Box as ground  */
	//Playground* world = new Playground( odeHandle, osgHandle, osg::Vec3(20, 0.2, 0.2), 1, true);
	//Substance ground = odeHandle.substance;
        //std::cout << "Substance of Playground:	roughness:  " 	 << ground.roughness << std::endl;
	//std::cout<< "				slip:		"<< ground.slip << std::endl;
	//std::cout<< "				hardness:	"<< ground.hardness << std::endl;
	//std::cout<< "				elasticity:	"<< ground.elasticity << std::endl;
	//world->setGroundSubstance( ground );
	//std::cout << "PG: GroundThickness:  "<< world->getGroundThickness() << std::endl;
	//world->setGroundTexture("Images/whiteground.jpg");
	//world->setPose( osg::Matrix::translate(0,0,0)* osg::Matrix::rotate(M_PI/2.,1,0,0) );
	//world->setPosition( osg::Vec3(20,0,10) );
	//global.obstacles.push_back( world );

	setCameraMode( Static );       
	setCameraHomePos(Pos(3.77046, 29.2309, 36.8821),  Pos(173.891, -53.7632, 0));
	setCameraHomePos(Pos(-34.1465, 17.9342, 21.7339),  Pos(-115.13, -28.0916, 0));
	RobInitPos = Pos(0,0,0);
	} break;
    case OP: { 
	OctaPlayground* world = new OctaPlayground( odeHandle, osgHandle, Pos(15,0.2,0.5), 15, false);
	world->setPose( osg::Matrix::translate(0,0,0) );
	global.obstacles.push_back( world );
	setCameraMode( Static );
	setCameraHomePos(Pos(-37.9599, -9.93542, 23.9097),  Pos(-74.8937, -34.2922, 0));
	RobInitPos = Pos(0,0,0);


	/** additional wall -> corridor */
    	Playground* world1 = new Playground( odeHandle, osgHandle, osg::Vec3(7, 0.2, 0.2), 1, false);
	world1->setPosition( osg::Vec3(4,0,0) );
	global.obstacles.push_back( world1 );
	

	} break;  
     case TI: {
	TerrainGround* world = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/terrain_bumpInDip128.ppm", 
        				"Images/stripes.rgb", 30, 30, 4,
					OSGHeightField::Sum);
	world->setPose( osg::Matrix::translate(0,0,0.1) ); 
	global.obstacles.push_back( world );
	setCameraHomePos( Pos(-0.153522, 31.808, 35.3312), Pos(-179.292, -45.7944, 0) );
	setCameraMode( Static );
	RobInitPos = Pos(0,0,4);
	} break;
     case RU: {
	TerrainGround* world = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/threedips128.ppm", 
        				"Images/whiteground.jpg", 28, 28, 5,
					OSGHeightField::Sum);
	world->setPose( osg::Matrix::translate(0,0,0.1) ); 
	global.obstacles.push_back( world );
	setCameraHomePos( Pos(-0.153522, 31.808, 35.3312), Pos(-179.292, -45.7944, 0) );
	setCameraMode( Static );
	RobInitPos = Pos(0,0,4);
	} break;
     case PLA: {
	TerrainGround* world = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/plateau32.ppm", 
        				"terrains/dip128_flat_texture.ppm", 28, 28, 4,
					OSGHeightField::Sum);
	world->setPose( osg::Matrix::translate(0,0,0.1) ); //*osg::Matrix::rotate(M_PI, 1,0,0));
	global.obstacles.push_back( world );
	setCameraHomePos(Pos(-3.50842, 35.7864, 39.3782),  Pos(-173.715, -46.8176, 0));
	setCameraMode( Static );
	RobInitPos = Pos(0,0,4);
	} break;
     case ELF: {
	TerrainGround* world = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/dip128_flat.ppm", 
        				"terrains/dip128_flat_texture.ppm", 50, 50, 8,
					OSGHeightField::Sum);
	world->setPose( osg::Matrix::translate(0,0,0.1) ); //*osg::Matrix::rotate(M_PI, 1,0,0));
	global.obstacles.push_back( world );
	setCameraHomePos( Pos(-0.153522, 31.808, 35.3312), Pos(-179.292, -45.7944, 0) );
	setCameraMode( Static );
	RobInitPos = Pos(0,0,0.2);
	} break;
     case EL: {
	TerrainGround* world = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/dip128.ppm", 
        				"terrains/dip128_texture.ppm", 20,20,3,  // 2, 2, 0.5,
					OSGHeightField::Red);
	world->setPose( osg::Matrix::translate(0,0,0.2) ); //*osg::Matrix::rotate(M_PI, 1,0,0));
	global.obstacles.push_back( world );
	setCameraHomePos(Pos(3.4179, 22.6125, 21.7825),  Pos(172.786, -44.2672, 0));
	setCameraMode( Static );
	RobInitPos = Pos(0,0,0.2);
	} break;
     case PT: {
	TerrainGround* world = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/3potential.ppm", 
        				"terrains/3potential_texture.ppm", 20, 20, 2); //120, 120, 5);
	world->setPose( osg::Matrix::translate(0,0,1) ); //*osg::Matrix::rotate(M_PI, 1,0,0));
	global.obstacles.push_back( world );
	setCameraHomePos( Pos(-0.153522, 31.808, 35.3312), Pos(-179.292, -45.7944, 0) );
	setCameraMode( Static );
	RobInitPos = Pos(0,0,3);
	} break;
     case TB: {
	TerrainGround* world = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/threebumps.ppm", 
        				//"terrains/threebumps.ppm", 20, 20, 2,
        				"Images/stripes.rgb", 20, 20, 2,
					OSGHeightField::Sum);
	world->setPose( osg::Matrix::translate(0,0,1) ); //*osg::Matrix::rotate(M_PI, 1,0,0));
	global.obstacles.push_back( world );
	TerrainGround* world1 = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/threebumps.ppm", 
        				"Images/stripes.rgb", 20, 20, 2,
					OSGHeightField::Red);
	world1->setPose( osg::Matrix::translate(20,0,1) *osg::Matrix::rotate(0, 0, M_PI/4, 0));
	global.obstacles.push_back( world1 );
	TerrainGround* world2 = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/threebumps.ppm", 
        				"Images/stripes.rgb", 20, 20, 2,
					OSGHeightField::Red);
	world2->setPose( osg::Matrix::translate(0,20,1) *osg::Matrix::rotate(0, 0, 0, 0));
	global.obstacles.push_back( world2 );
	TerrainGround* world3 = new TerrainGround( odeHandle, osgHandle, 
        				"terrains/threebumps.ppm", 
        				"Images/stripes.rgb", 20, 20, 2,
					OSGHeightField::Red);
	world3->setPose( osg::Matrix::translate(20,20,1) *osg::Matrix::rotate(0, 0, 0, 0));
	global.obstacles.push_back( world3);
	setCameraHomePos(Pos(8.20536, 62.9326, 51.9464),  Pos(-179.292, -45.7944, 0));
	setCameraMode( Static );
	RobInitPos = Pos(0,0,3);
	} break;
     case HG: {
	TerrainGround* world = new TerrainGround( odeHandle, osgHandle, 
        				//---"terrains/macrospheresLMH_256.ppm", 
        				//---"terrains/macrospheresLMH_256.ppm", 128, 128, 3,
        				//"terrains/rauschen32.ppm",  // not tough
        				//"terrains/rauschen32.ppm", 128, 128, 3,
        				//"terrains/whirl32.ppm", 
        				//"terrains/whirl32.ppm", 128, 128, 3,
        				"./environments/white.ppm",
        				"Images/whiteground.jpg", 20, 20, 0.,
        				//"terrains/zoo_landscape2.ppm", 
        				//"terrains/zoo_landscape2.ppm", 128, 128, 3,
					OSGHeightField::Red);
	world->setPose( osg::Matrix::translate(0,0,2) ); 
	global.obstacles.push_back( world );
	//TerrainGround* world2 = new TerrainGround( odeHandle, osgHandle, 
        //				"./environments/white.ppm",
        //				"Images/whiteground.jpg", 5, 5, 0.,
	//				OSGHeightField::Red);
	//world2->setPose( osg::Matrix::translate(5,0,0.1) ); 
	//global.obstacles.push_back( world2 );
	//TerrainGround* world1 = new TerrainGround( odeHandle, osgHandle, 
        //				"./environments/white.ppm",
        //				"Images/whiteground.jpg", 5, 5, 0.,
	//				OSGHeightField::Red);
	//world1->setPose( osg::Matrix::translate(5,5,0.1) ); 
	//global.obstacles.push_back( world1 );
	//TerrainGround* world3 = new TerrainGround( odeHandle, osgHandle, 
        //				"./environments/white.ppm",
        //				"Images/whiteground.jpg", 5, 5, 0.,
	//				OSGHeightField::Red);
	//world3->setPose( osg::Matrix::translate(0,5,0.1) ); 
	//global.obstacles.push_back( world3 );

	//setCameraHomePos( Pos(-0.153522, 31.808, 35.3312), Pos(-179.292, -45.7944, 0) );
	setCameraHomePos(Pos(0.958547, 39.3372, 41.4114),  Pos(-179.292, -45.7944, 0));
	setCameraMode( Static );
	RobInitPos = Pos(0,0,3); 
	} break;
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
	//case 'b' : controller->setRandomPhi(); break;
	//case 'n' : controller->setRandomU(); break;
	//case 'm' : controller->setRandomX(10.); break;
	case 'r' : controller->setRandomAll(10.); break;
	case 'm' : robot->moveToPosition(Pos(0,0,2.25)); break;
	case 'M' : robot->moveToPosition(Pos(20,0,10.25)); break;
        case 't' : agent->setTrackOptions(TrackRobot(true, true, true, false)); 
                   std::cout<< "track file: open or close " << std::endl; break;
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

