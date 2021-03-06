#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/meshground.h>
#include <ode_robots/randomobstacles.h>
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
  RandomObstacles* RandObstacle; 
  bool twoSpheres = false;

  enum robotType {TypeBarrel, TypeSphere};
  robotType type= TypeSphere;
  
  /** Environments: NO: flat ground
  **	if the last argument in the constructor is false, no additional ground is added
  **		    PG: playground fenced by a wall
  **		    OP: polygonal playground fenced by a wall
  **  	TerrainGround objects, their ground is created by using a .ppm file
  **		    HG: flat ground created by white .ppm file
  **		    TB: three basins 
  **		    PT: three potentials different depth
  **		    EL: round or elliptical potential
  **		    ELF: same but different shape
  **		    PLA: plateau
  **		    TI: trench
  **		    RU: 3 little trenchs  */
  enum Env { NO, PG, OP, HG, TB, PT, EL, ELF, PLA, TI, RU, maze};
  Env env = maze;

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
    /** The default values of substance is defined in odeHandle.substance  */
    /** odeHandle Substance: roughness:  0.8,  slip:  0.01,   hardness:  40,  elasticity:  0.5  */
    //setGroundSubstance( Substance::getPlastic(0.8) );
    Substance GroundSub = getGroundSubstance(); 
    std::cout << "GroundSubstance:	 roughness:  " << GroundSub.roughness << std::endl;
    std::cout << "			 slip:	     " << GroundSub.slip << std::endl;
    std::cout << "			 hardness:   " << GroundSub.hardness << std::endl;
    std::cout << "			 elasticity: " << GroundSub.elasticity << std::endl;
 

    /********** ENVIRONMENT **********/
    /** to create different environments as walls */
    /** be carefull when using different grounds, this can modify the locomotions */
    /** different calculation of contact points is the most likely reason */
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


    /****** RANDOM OBSTACLES ********/
    /** to enable online generation of random obstacles with certain characteristics */
	RandomObstaclesConf randConf = RandomObstacles::getDefaultConf();  
    randConf.pose = osg::Matrix::translate(0,0,0);
    randConf.area = Pos(4,4,2);
    randConf.minSize = Pos(.4,.4,.4);
    randConf.maxSize = Pos(1.5,1.5,.8);
    randConf.minDensity = 1;
    randConf.maxDensity = 10;
    RandObstacle = new RandomObstacles(odeHandle, osgHandle, randConf);
    /** Generation an placing Objects */
    //int num_randObs = 7; 
    //for (int i=0; i< num_randObs; i++){
    //	RandObstacle->spawn(RandomObstacles::Box, RandomObstacles::Foam);
    //	global.obstacles.push_back( RandObstacle );
    //}

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
       robot->place (osg::Matrix::rotate(M_PI/4, 1,1,0)*osg::Matrix::translate(0,0,1)); 
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
       /** to change the substance/material of the robot. be careful: material influences the behaviour */
       OdeHandle myHandle = odeHandle;    /** default: plastic with roughness= 0.8 */
       //myHandle.substance.toMetal(0.5);   /** roughness [0.1,1], very hard, elastic, slip */
       //myHandle.substance.toRubber(50);   /** hardness [5,50], high roughness, no slip, very elastic */
       robot = new SphereRobot( myHandle, osgHandle.changeColor(Color(0.,0.,1.)), sconf, 
				"Sphere", global.odeConfig, 0.4);
       robot->addSensor(std::make_shared<SpeedSensor>( 1, SpeedSensor::Translational ),Attachment(-1));
       //robot->place(osg::Matrix::translate(0,0,20));
       robot->place(RobInitPos);
       controller = new STSPController( global.odeConfig );

       /** plotTypes=8 == Noise, Default == Controller */
       One2OneWiring* wiring = new One2OneWiring( new WhiteNormalNoise() );  // new One2OneWiring( new ColorUniformNoise(1.) );

       agent = new OdeAgent( globalData );
       agent->init( controller, robot, wiring );
       global.agents.push_back( agent );
       global.configs.push_back( controller );
       global.configs.push_back( robot );
    }
    if(twoSpheres == true){
       SphereRobotConf sconf = SphereRobot::getDefaultConf();
       sconf.diameter = 0.5;
       sconf.motorpowerfactor = 120;
       /** to change the substance/material of the robot. be careful: material influences the behaviour */
       OdeHandle myHandle = odeHandle;    /** default: plastic with roughness= 0.8 */
       //myHandle.substance.toMetal(0.5);   /** roughness [0.1,1], very hard, elastic, slip */
       //myHandle.substance.toRubber(50);   /** hardness [5,50], high roughness, no slip, very elastic */
       OdeRobot* robot2 = new SphereRobot( myHandle, osgHandle.changeColor(Color(0.,1.,1.)), sconf, 
				"Sphere2", global.odeConfig, 0.4);
       robot2->addSensor(std::make_shared<SpeedSensor>( 1, SpeedSensor::Translational ),Attachment(-1));
       //robot->place(osg::Matrix::translate(0,0,20));
       robot2->place(Pos(1,0,0));
       STSPController* controller2 = new STSPController( global.odeConfig );

       /** plotTypes=8 == Noise, Default == Controller */
       One2OneWiring* wiring2 = new One2OneWiring( new WhiteNormalNoise() );  // new One2OneWiring( new ColorUniformNoise(1.) );

       OdeAgent* agent2 = new OdeAgent( globalData );
       agent2->init( controller2, robot2, wiring2 );
       global.agents.push_back( agent2 );
       global.configs.push_back( controller2 );
       global.configs.push_back( robot2 );
		TrackRobot* TrackOpt2 = new TrackRobot(false, false, false, true); 
    	TrackOpt2->conf.displayTraceDur = 200; /** length of track line */
    	TrackOpt2->conf.displayTraceThickness = 0.0; /** thickness of track line, if 0 then it is a line */
    	TrackOpt2->conf.displayTraceThickness = 0.0; /** thickness of track line, if 0 then it is a line */
		agent2->addTracking(0, *TrackOpt2, Color(0.,1.,1.));
		agent2->setTrackOptions( *TrackOpt2 );
    }

    /** tracking: ( trackPos, trackSpeed, trackOrientation, displayTrace, scene  = "char", interval ) */
    /** more options in selforg/utils/trackrobot.h */
    /** if 1 of the first 3 arguments  == true:  log file with values is created */
    if(track == true){
		TrackRobot* TrackOpt = new TrackRobot(false, false, false, true); 
    	TrackOpt->conf.displayTraceDur = 200; /** length of track line */
    	TrackOpt->conf.displayTraceThickness = 0.0; /** thickness of track line, if 0 then it is a line */
		agent->setTrackOptions( *TrackOpt );
    }
  };




  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(!pause){
       if(friction>0){
          OdeRobot* robot = globalData.agents[0]->getRobot();
          Pos vel = robot->getMainPrimitive()->getAngularVel();
          robot->getMainPrimitive()->applyTorque(-vel*friction);
		  if(twoSpheres==true){
            OdeRobot* robot2 = globalData.agents[1]->getRobot();
            Pos vel = robot2->getMainPrimitive()->getAngularVel();
            robot2->getMainPrimitive()->applyTorque(-vel*friction);
		  }
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
    case maze: {
	srand(time(NULL));

	double widthX = 20;
	int num_wall = 5;
    	Playground* maze = new Playground( odeHandle, osgHandle, osg::Vec3(widthX, 0.2, 0.3), 1, false);
	maze->setPosition( osg::Vec3(0,0,0) );
	global.obstacles.push_back( maze );

	Box* wall[num_wall];
	wall[0] = new Box( 10, 0.2, 0.3 );
	wall[0]->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	wall[0]->setPosition( Pos(5,2,0) ); 

	wall[1] = new Box( 0.2, 8, 0.3 );
	wall[1]->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	wall[1]->setPosition( Pos(2,2,0) ); 
	
	wall[2] = new Box( 15, 0.2, 0.3 );
	wall[2]->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	wall[2]->setPosition( Pos(-2.5,-6,0) ); 

	wall[3] = new Box( 0.2, 10, 0.3 );
	wall[3]->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	wall[3]->setPosition( Pos(-4,3,0) ); 

	wall[4] = new Box( 6, 0.2, 0.3 );
	wall[4]->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	wall[4]->setPosition( Pos(-1,-2,0) ); 

	//for(int i=0; i<num_wall; i++){
	//    double wwidth = rand()%5 +5 ;
	//    double wlen = 0.2;
	//    if( rand()%2 ==1 ) wall[i] = new Box( wwidth, wlen, 0.3 );
	//    else  wall[i] = new Box( wlen, wwidth, 0.3 );
	//    double posx = rand()%15*1.5 - widthX/2;
	//    double posy = rand()%15*2   - widthX/2;
	//    wall[i]->init( odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw );
	//    wall[i]->setPosition( Pos(posx,posy,0) ); 
	//}	

	setCameraMode( Static );        
	//setCameraHomePos(Pos(-0.403611, 41.6779, 46.5942),  Pos(179.949, -51.1891, 0));
	//setCameraHomePos(Pos(0.0919222, 18.978, 33.1766),  Pos(179.697, -62.2167, 0));
	//setCameraHomePos(Pos(0.063416, 0.374084, 42.2245),  Pos(-179.806, -90.2157, 0));
	setCameraHomePos(Pos(0., -0.6, 42.),  Pos(0., -90., 90.));

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
	OctaPlayground* world = new OctaPlayground( odeHandle, osgHandle, Pos(5,0.2,0.5), 15, false);
	world->setPose( osg::Matrix::translate(0,0,0) );
	global.obstacles.push_back( world );
	//setCameraMode( Static );
	//setCameraHomePos(Pos(-7.60245, -1.26259, 16.1609),  Pos(-75.9145, -65.364, 0));
	setCameraHomePos(Pos(-0.0172261, 1.94153, 0.446375),  Pos(-174.344, -9.53503, 0));
	setCameraMode( Follow );
	//setCameraHomePos(Pos(-37.9599, -9.93542, 23.9097),  Pos(-74.8937, -34.2922, 0));
	RobInitPos = Pos(0,0,0);


	/** additional wall -> corridor */
    	//Playground* world1 = new Playground( odeHandle, osgHandle, osg::Vec3(7, 0.2, 0.2), 1, false);
	//world1->setPosition( osg::Vec3(4,0,0) );
	//global.obstacles.push_back( world1 );

	/** create a box for testing purposes */
	//auto* box = new Box(1,1,0.5);
	//box->init( odeHandle, 2, osgHandle,  Primitive::Geom | Primitive::Draw | Primitive::Body );
	//box->setSubstance(Substance::getPlastic(0.8));
	//box->setPosition(Pos(0,0,0.2)); //member function of Box class
	//global.obstacles.push_back( box );
 

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
        //case 'a' : controller->increaseA(-0.02);
        //           std::cout << "new a:  "<<controller->getParam("a") << std::endl;
        //           break;
        //case 'A' : controller->increaseA(0.02);
        //           std::cout << "new a:  "<<controller->getParam("a") << std::endl;
        //           break;
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
        //case 'g' : controller->increaseGamma(-0.1);
        //           std::cout<< "new gamma:  "<<controller->getParam("gamma")<< std::endl;
        //           break;
        //case 'G' : controller->increaseGamma(0.1);
        //           std::cout<< "new gamma:  "<<controller->getParam("gamma")<< std::endl;
        //           break;
	case 'r' : controller->setRandomAll(10.); break;
	//case 'b' : controller->setRandomPhi(); break;
	//case 'n' : controller->setRandomU(); break;
	//case 'm' : controller->setRandomX(10.); break;
	case 'm' : robot->moveToPosition(Pos(0,0,2.25)); break;
	case 'M' : robot->moveToPosition(Pos(20,0,10.25)); break;
	case 't' : agent->setTrackOptions(TrackRobot(true, true, true, false,"",100)); 
	           std::cout<< "Track file: open " << std::endl; break;
	case 'T' : agent->setTrackOptions(TrackRobot(false, false, false, false)); 
	           std::cout<< "Track file: close " << std::endl; break;
	case 's' : agent->setTrackOptions(TrackRobot(false, false, false, true)); 
	           std::cout<< "drawing track line " << std::endl; break;
	case 'S' : agent->stopTracking();
	           std::cout<< "Stop tracking" << std::endl; break;
	case 'q' : RandObstacle->spawn(); 
		   globalData.obstacles.push_back( RandObstacle );
		   break;
	case 'Q' : RandObstacle->remove(); 
	    	   globalData.obstacles.pop_back(); 
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
  sim.setCaption("Robot with STSP controlling (lpzrobots Simulator)   Martin 2016");
  return sim.run(argc, argv) ? 0 : 1;
}

