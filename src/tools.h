#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
  * A helper method to calculate Jacobians.
  */
    Eigen::VectorXd CalculateNonlinearH(const Eigen::VectorXd& x_state);

  // From: http://stackoverflow.com/a/29871193/1321129
  /* change to `float/fmodf` or `long double/fmodl` or `int/%` as appropriate */
  /* wrap x -> [0,max) */
  double wrapMax(double x, double max);

  /* wrap x -> [min,max) */
  double wrapMinMax(double x, double min, double max);

};

#endif /* TOOLS_H_ */
