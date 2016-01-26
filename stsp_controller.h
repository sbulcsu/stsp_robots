
#include <stdio.h>
#include <selforg/abstractcontroller.h>
#include <vector>

#include <ode_robots/odeconfig.h>

class STSPController : public AbstractController {
public:


  STSPController(const lpzrobots::OdeConfig& odeconfig );

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual int getSensorNumber() const {return number_sensors;}

  virtual int getMotorNumber() const {return number_motors;}

  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);

  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);


  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }

  double y(double x);
  double U(double y);
  double PHI(double y, double u);

protected:
  const lpzrobots::OdeConfig& odeconfig;


  std::string name;
  double a; 
  double b;
  double stepsize; 

  int number_sensors;
  int number_motors;

  double w_0;
  double z_0;
  double T_u;
  double T_phi;
  double U_max;
  double gamma;
  double r; 

  double sensor0;
  double sensor1;

  double x0_old;
  double x1_old;
  double x0_new;
  double x1_new;

  double y0_old;
  double y1_old;
  double y0_new;
  double y1_new;
  
  double u0_old;
  double u1_old;
  double u0_new;
  double u1_new;

  double phi0_old;
  double phi1_old;
  double phi0_new;
  double phi1_new;

};



