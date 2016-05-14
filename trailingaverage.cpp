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
#include <string>
#include <math.h>

#include <ode_robots/primitive.h>
#include "trailingaverage.h"
#include <ode_robots/mathutils.h>


using namespace matrix;

namespace lpzrobots {

  TrailingAverage::TrailingAverage( double scale, double maxSpeed, const OdeConfig& odeconfig, Mode mode, short dimensions )
    : timescale(scale), maxSpeed(maxSpeed), odeconfig(odeconfig), mode(mode), dimensions(dimensions) {
    average[0] = 0;
    average[1] = 0;
    average[2] = 0;
    averageXYZ = 0;
    stepsize = odeconfig.simStepSize;
    own=0;
    std::string name = "TrailingAverage";
    switch(mode){
        case SingleValue: name += "OneDim";
                          break;  
        //case Y:   name += "Y";
        //          break;  
        //case Z:   name += "Z";
        //          break;  
        case VectorXYZ: name += "XYZ";
                        break;  
    }
    //TODO
    setBaseInfo(SensorMotorInfo(name).changequantity(SensorMotorInfo::Velocity));
#if (__GNUC__ > 4 ) || (__GNUC__ == 4 && __GNUC_MINOR__ > 7)
    setNamingFunc([dimensions](int index) {return dimensions2String(dimensions).substr(index,1);});
#endif
  }


  void TrailingAverage::init(Primitive* own, Joint* joint){
    this->own = own;
  }

  int TrailingAverage::getSensorNumber() const{
    //return (dimensions & X) + ((dimensions & Y) >> 1)  + ((dimensions & Z) >> 2);
    return 3*(mode == SingleValue) + (mode == VectorXYZ);
  }

  bool TrailingAverage::sense(const GlobalData& globaldata) { 
      stepsize = odeconfig.simStepSize;//*odeconfig.controlInterval;
      return true; 
  }

  //TODO   
  std::list<sensor> TrailingAverage::getList() const {
     Matrix& m = getSenseMatrix();
    if( mode==SingleValue) return selectrows(m,3); //dimensions falsch hier. entweder 3 oder 1
    else return selectrows(m,1); 
  }

  int TrailingAverage::get(sensor* sensors, int length) const{
    //const Matrix& m = getSenseMatrix()*(1.0/maxSpeed);
    Matrix& m = getSenseMatrix();
    //if(dimensions == (X | Y | Z))
    //if( mode==SingleValue)
    //  return m.convertToBuffer(sensors, length);
    //else{
    //  return selectrows(sensors, length, m, dimensions);
    //}
    return 0;
  }

  //Matrix TrailingAverage::getSenseMatrix() const {
  Matrix TrailingAverage::getSenseMatrix(){
    assert(own);
    assert(own->getBody());
    Matrix speed;
    Matrix ave;
    speed.set(3,1, dBodyGetLinearVel(own->getBody()));
    switch(mode){
    case SingleValue:
      // berechne hier den neuen average für alle dimensionen
      for(int i=0; i<3; i++){
        average[i] = calcTrailingAve( speed.val(i,0), average[i] );
      }
      ave.set( 3,1, average );
      break;
    case VectorXYZ:
      // berechne zuerst die länge des vectors vx,vy,vz
      double vecLen = sqrt( speed.val(0,0)*speed.val(0,0) + 
                      speed.val(1,0)*speed.val(1,0) +
                      speed.val(2,0)*speed.val(2,0) );
      // dann trailing average
      averageXYZ = calcTrailingAve( vecLen, averageXYZ);
      ave.set( 1,1, averageXYZ );
      break;
    return ave;
  }
  }

  double TrailingAverage::calcTrailingAve(double sensor, double var){
    double oldave = var;
    return oldave + stepsize/timescale * (sensor - oldave);
  }

}



