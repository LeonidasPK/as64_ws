/**
 * Copyright (C) 2017 CAN_SYS
 */


/** Canonical system class
 * 
 * Implements a 1st order exponentially decaying canonical system for a DMP.
 * The canonical system is defined as:
 * 	tau*dx = -a_x*x
 * where x is the phase variable and a_x the decay term and tau is a scaling factor defining the duration of the motion.
 *
 */

#ifndef DMP_EXPONENTIAL_CANONICAL_SYSTEM_64_H
#define DMP_EXPONENTIAL_CANONICAL_SYSTEM_64_H

#include <DMP_lib/CanonicalSystem/CanonicalSystem.h>

namespace as64
{
	
class ExpCanonicalSystem:public CanonicalSystem
{
public:
  
  ExpCanonicalSystem();

  void set_can_sys_params(double x_end);

  arma::vec get_derivative(const arma::vec &X);

  arma::mat get_continuous_output(const arma::rowvec &t, double x0);

private:
  double a_x;
};

}  // namespace as64

#endif  // DMP_EXPONENTIAL_CANONICAL_SYSTEM_64_H