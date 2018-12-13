#ifndef PID_H
#define PID_H
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients

  double Kp;
  double Ki;
  double Kd;
  */
  std::vector<double> Kpid;

  /* twiddle parameters
  double dKp;
  double dKi;
  double dKd;
  */
  std::vector<double> dKpid;
  double tol;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double dKp=0.05, double dKi=0.00000, double dKd=0.15, double tolerance=0.01);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /* Twiddle, adjust Kp Ki Kd coefficiency */
  void Twiddle(double cte);
};

#endif /* PID_H */
