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
#ifndef __AVESPEEDSENSOR_H
#define __AVESPEEDSENSOR_H

#include <ode_robots/sensor.h>
#include <ode_robots/odeconfig.h>

namespace lpzrobots {

  /** Class for trailing average sensors
      Calculates the mean of a value on the run
  */
  class TrailingAverage : public Sensor {
  public:

    /// Sensor mode
    enum Mode { SingleValue, VectorXYZ };

    /**
       @param maxSpeed maximal speed that is expected used for normalisation of sensor value
       @param dimensions bit mask for the dimensions to sense. Default: X | Y | Z (all dimensions)
       @see Dimensions
     */
    TrailingAverage( double scale, double maxSpeed, const OdeConfig& odeconfig, Mode mode = SingleValue, short dimensions = X | Y | Z );
    virtual ~TrailingAverage() {}

    virtual void init(Primitive* own, Joint* joint = 0);
    virtual int getSensorNumber() const;

    virtual bool sense(const GlobalData& globaldata);
    virtual std::list<sensor> getList() const;
    virtual int get(sensor* sensors, int length) const;
  protected:
    //matrix::Matrix getSenseMatrix() const;
    matrix::Matrix getSenseMatrix();
    double calcTrailingAve(double sensor, double var);

    void setTimeScale( double scale){ timescale = scale; }
    double getTimeScale(){ return timescale; }

    
  protected:
    double timescale;
    double stepsize;
    double average[3] ;
    double averageXYZ ;

    const OdeConfig& odeconfig;

    double maxSpeed;
    Mode mode;
    short dimensions;
    Primitive* own;
  };


}

#endif
