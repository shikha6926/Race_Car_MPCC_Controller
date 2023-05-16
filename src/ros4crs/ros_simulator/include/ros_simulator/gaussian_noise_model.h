#ifndef ROS_SIMULATOR_GAUSSIAN_NOISE_MODEL_H
#define ROS_SIMULATOR_GAUSSIAN_NOISE_MODEL_H

#include "ros_simulator/common/noise_model.h"

// Taken from https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c,
#include <Eigen/Dense>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

namespace Eigen
{
namespace internal
{
template <typename Scalar>
struct scalar_normal_dist_op
{
  static boost::mt19937 rng;                        // The uniform pseudo-random algorithm
  mutable boost::normal_distribution<Scalar> norm;  // The gaussian combinator

  EIGEN_EMPTY_STRUCT_CTOR(scalar_normal_dist_op)

  template <typename Index>
  inline const Scalar operator()(Index, Index = 0) const
  {
    return norm(rng);
  }
};

template <typename Scalar>
boost::mt19937 scalar_normal_dist_op<Scalar>::rng;

template <typename Scalar>
struct functor_traits<scalar_normal_dist_op<Scalar>>
{
  enum
  {
    Cost = 50 * NumTraits<Scalar>::MulCost,
    PacketAccess = false,
    IsRepeatable = false
  };
};
}  // end namespace internal
}  // end namespace Eigen

namespace ros_simulator
{
class GaussianNoiseModel : NoiseModel
{
private:
  Eigen::VectorXd mean_;
  bool has_mean = false;

public:
  GaussianNoiseModel(int seed)  // Option 1: Set a seed, mean will default to zero
  {
    Eigen::internal::scalar_normal_dist_op<double>::rng.seed(seed);  // Seed the rng
  };

  GaussianNoiseModel(int seed, Eigen::VectorXd mean) : GaussianNoiseModel(seed)  // Option 2: Set a seed and a mean
  {
    mean_ = mean;
    has_mean = true;
  };

  Eigen::MatrixXd sampleNoiseFromCovMatrix(const Eigen::MatrixXd& Q) override
  {
    int size = Q.rows();                                   // Dimensionality (rows)
    int nn = 1;                                            // How many samples (columns) to draw
    Eigen::internal::scalar_normal_dist_op<double> randN;  // Gaussian functor

    if (!has_mean)
    {
      mean_.setZero(size);
      has_mean = true;
    }

    // Define mean and covariance of the distribution
    Eigen::MatrixXd covar = Q;

    Eigen::MatrixXd normTransform(size, size);

    Eigen::LLT<Eigen::MatrixXd> cholSolver(covar);

    // We can only use the cholesky decomposition if
    // the covariance matrix is symmetric, pos-definite.
    // But a covariance matrix might be pos-semi-definite.
    // In that case, we'll go to an EigenSolver
    if (cholSolver.info() == Eigen::Success)
    {
      // Use cholesky solver
      normTransform = cholSolver.matrixL();
    }
    else
    {
      // Use eigen solver
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
      normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }
    return (normTransform * Eigen::MatrixXd::NullaryExpr(size, nn, randN)).colwise() + mean_;
  };
};
};  // namespace ros_simulator
#endif
