/**
* Copyright (C) 2017 DMP
*/


/** DMP class
*  Implements an 1-D bio-inspired DMP. 
*
*/

#ifndef DYNAMIC_MOVEMENT_PRIMITIVES_BIO_2002_H
#define DYNAMIC_MOVEMENT_PRIMITIVES_BIO_2002_H

#include <DMP_lib/DMP/DMP.h>

namespace as64
{

class DMPBio:public DMP
{
public:
	
	DMPBio();

  DMPBio(int N_kernels, double a_z, double b_z, std::shared_ptr<CanonicalSystem> cs_ptr, double std_K=1, bool USE_GOAL_FILT=false, double a_g=0);

  // Returns the shape-attractor of the DMP
  double shape_attractor(const arma::vec X, double g, double y0);

private:

	// calculates the desired force for the demonstraded path
  void calculate_Fd(const arma::rowvec &yd_data, const arma::rowvec &dyd_data, const arma::rowvec &ddyd_data, arma::mat &u, arma::rowvec &g, arma::rowvec &y0, arma::rowvec &Fd);

};

}  //as64

#endif  // DYNAMIC_MOVEMENT_PRIMITIVES_BIO_2002_H
