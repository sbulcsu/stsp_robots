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

STSPController::STSPController(const OdeConfig& odeconfig )
  : AbstractController("STPController", "1.0"), odeconfig(odeconfig)
  {
   number_sensors = 0;
   number_motors = 0;
};

void STSPController::init(int sensornumber, int motornumber, RandGen* randGen){
     number_sensors = sensornumber;
     number_motors = motornumber;
     cout<< " Number of motors: "  << number_motors << endl;

     addParameterDef("a", &a,0.4);
     addParameterDef("b", &b,0);   
     addParameterDef("r", &r,0.5, "scaling factor of the sigmoidal function (<=1)");
     //addParameterDef("targetpos", &targetpos,0, "if not 0, this is the target of the pid-cont");
     addParameterDef("w_0", &w_0, 100., "");
     addParameterDef("z_0", &z_0, -300., "");
     addParameterDef("T_u", &T_u, 0.3, "");
     addParameterDef("T_phi", &T_phi, 0.6, "");
     addParameterDef("U_max", &U_max, 1., "");
     addParameterDef("gamma", &gamma, 10, "");
     //addParameter("x0", &x0);
     //addParameter("x1", &x1);

     x0_old = 1;
     x1_old = 1;  
     x0_new = 1;
     x1_new = 1;  
     addInspectableValue("x0", &x0_old, "  ");
     addInspectableValue("x1", &x1_old, "  ");
     u0_old = 1;
     u1_old = 1;
     u0_new = 1;
     u1_new = 1;
     addInspectableValue("u0", &u0_old, "  ");
     addInspectableValue("u1", &u1_old, "  ");
     phi0_old = 1.;
     phi1_old = 1.;
     phi0_new = 1.;
     phi1_new = 1.;
     addInspectableValue("phi0", &phi0_old, "  ");
     addInspectableValue("phi1", &phi1_old, "  ");
     y0_old = 1;
     y1_old = 1;
     y0_new = 1;
     y1_new = 1;
     addInspectableValue("y0", &y0_old, "  ");
     addInspectableValue("y1", &y1_old, "  ");
     sensor0 = 0;
     sensor1 = 0;
     addInspectableValue("sensor0", &sensor0, "  ");
     addInspectableValue("sensor1", &sensor1, "  ");

};

void STSPController::step(const sensor* sensors, int sensornumber,
                          motor* motors, int motornumber) {
    stepsize = odeconfig.simStepSize*odeconfig.controlInterval;

    sensor0 = sensors[0];
    sensor1 = sensors[1];

    u0_new = u0_old+ ((U(y0_old)- u0_old)/ T_u)* stepsize;
    u1_new = u1_old+ ((U(y1_old)- u1_old)/ T_u)* stepsize;
    
    phi0_new = phi0_old+ (( PHI(y0_old,u0_old) - phi0_old )/ T_phi)* stepsize; 
    phi1_new = phi1_old+ (( PHI(y1_old,u1_old) - phi1_old )/ T_phi)* stepsize;


    x0_new = x0_old + (- gamma* x0_old 
                       + w_0* ( (sensor0/r)+ 0.5 )
                       + z_0* u1_old * phi1_old* y1_old )*  stepsize; 

    x1_new = x1_old + (- gamma* x1_old 
                       + w_0* ( (sensor1/r)+ 0.5 )
                       + z_0* u0_old * phi0_old* y0_old )*  stepsize; 
    y0_new = y(x0_new);
    y1_new = y(x1_new);

    motors[0] =  r* (y0_new- 0.5);
    motors[1] =  r* (y1_new- 0.5);

cout << "("<<  motors[0]<< ","<< motors[1]<< ")     " << "("<<  sensors[0]<< ","<< sensors[1]<< ")     " << endl;

    /*** rewriting for next timestep ***/
    y0_old = y0_new;
    y1_old = y1_new;
    x0_old = x0_new;
    x1_old = x1_new;
    phi0_old = phi0_new;
    phi1_old = phi1_new;


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



void STSPController::stepNoLearning(const sensor* sensors, int number_sensors,
                                    motor* motors, int number_motors) {

};
