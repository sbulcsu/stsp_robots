
#include <stdio.h>
#include <cmath>
#include <assert.h>
#include <selforg/controller_misc.h>
#include <ode_robots/simulation.h>
#include <vector>
#include <fstream>

#include "stsp_controller.h"

using namespace std;
using namespace lpzrobots;

STSPController::STSPController(const OdeConfig& odeconfig, double diameter, double prange ) 
  : AbstractController("STPController", "1.0"), odeconfig(odeconfig)
  {
   number_sensors = 0;
   number_motors = 0;
   radius = diameter/2.; 
   pendularrange = prange; 
};

void STSPController::init(int sensornumber, int motornumber, RandGen* randGen){
     number_sensors = sensornumber;
     number_motors = motornumber;
     cout<< " Number of motors: "  << number_motors << endl;

     addParameterDef("a", &a,0.4);
     addParameterDef("b", &b,0);   
     addParameterDef("r", &r,1., "scaling factor of the sigmoidal function (<=1)");
     addParameterDef("w_0", &w_0, 100., "");
     addParameterDef("z_0", &z_0, -300., "");
     addParameterDef("T_u", &T_u, 0.3, "");
     addParameterDef("T_phi", &T_phi, 0.6, "");
     addParameterDef("U_max", &U_max, 1., "");
     addParameterDef("gamma", &gamma, 10, "");

     neuron.resize(number_motors);
     for(int i = 0; i<number_motors; i++){
	 neuron[i].x_old = 1;
	 neuron[i].x_new = 1;
	 neuron[i].y_old = 1;
	 neuron[i].y_new = 1;
	 neuron[i].u_old = 1;
	 neuron[i].u_new = 1;
	 neuron[i].phi_old = 1;
	 neuron[i].phi_new = 1;
	 neuron[i].sensor = 1;
	 addInspectableValue("x"+ itos(i), &neuron[i].x_old, "  ");
	 addInspectableValue("u"+ itos(i), &neuron[i].u_old, "  ");
	 addInspectableValue("phi"+ itos(i), &neuron[i].phi_old, "  ");
	 addInspectableValue("y"+ itos(i), &neuron[i].y_old, "  ");
	 addInspectableValue("sensor"+ itos(i), &neuron[i].sensor, "  ");
	 neuron[i].x_a = 0; 
         addInspectableValue("x_a"+ itos(i), &neuron[i].x_a ,  "   ");  
     }
};

void STSPController::step(const sensor* sensors, int sensornumber,
                          motor* motors, int motornumber) {
     stepsize = odeconfig.simStepSize*odeconfig.controlInterval;

     for(int i= 0; i< number_motors; i++){
         neuron[i].sensor  = sensors[i];
         neuron[i].x_a     = neuron[i].sensor* pendularrange *radius; 
         neuron[i].u_new   = neuron[i].u_old +
         		     ((U(neuron[i].y_old)-neuron[i].u_old)/T_u)*stepsize; 
         neuron[i].phi_new = neuron[i].phi_old +
         	             ((PHI(neuron[i].y_old,neuron[i].u_old)-neuron[i].phi_old )/T_phi)
         		     *stepsize; 
         neuron[i].x_new   = neuron[i].x_old + 
         		     (- gamma* neuron[i].x_old 
                             + w_0* mtargetInv( neuron[i].sensor ))   // durch funktion ersetzen
         		     *stepsize;
         for(int j=0; j<number_motors; j++){
	     if (i!=j) {
             neuron[i].x_new += (z_0* neuron[j].u_old* neuron[j].phi_old* neuron[j].y_old)* stepsize;
	     }
         }
         neuron[i].y_new    = y( neuron[i].x_new );
         motors[i]          = mtarget( neuron[i].y_new );
     }
     
     /*** rewriting for next timestep ***/
     for(int i=0; i< number_motors; i++){
         neuron[i].y_old = neuron[i].y_new;
         neuron[i].x_old = neuron[i].x_new;
         neuron[i].u_old = neuron[i].u_new;
         neuron[i].phi_old = neuron[i].phi_new;
     }
};

double STSPController::y(double x){
       return 1. / (1. +exp(a*(b-x)));
};

double STSPController::U(double y){
       return  1.+ (U_max- 1.)* y;
};

double STSPController::PHI(double y, double u){
       return  1.- (u* y)/ U_max;
};

double STSPController::mtarget(double y){
       return r* ( 2*y - 1);
};

double STSPController::mtargetInv(double sensor){
       return sensor/(r*2) + 0.5 ;
};

void STSPController::stepNoLearning(const sensor* sensors, int number_sensors,
                                    motor* motors, int number_motors) {

};


