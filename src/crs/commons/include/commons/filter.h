#ifndef COMMONS_FILTER_H
#define COMMONS_FILTER_H

#include <iostream>
#include <vector>

/**
 * @brief Implementation of a Discrete-time and Statistical Signal Processing (DSSP) Filter
 *
 */
class Filter
{
public:
  /**
   * default constructor
   */
  Filter() : b_(1, 0), a_(1, 0), z_(1, 0){};

  /**
   * @brief Construct a new Filter object
   *
   * @param b numerator coefficients of digital Filter transfer function
   * @param a denominator coefficients of digital Filter transfer function
   */
  Filter(const std::vector<double>& b, const std::vector<double>& a) : b_(b), a_(a), z_(a.size(), 0){};  // NOLINT

  /**
   * @brief assignment operator
   */
  Filter(const Filter& t){};

  /**
   * @brief implement digital Filter using difference equation in
   *        direct II transposed structure form
   * @param sig new value of signal to Filter
   */
  double process(const double& sig);

private:
  std::vector<double> b_;
  std::vector<double> a_;
  std::vector<double> z_;
};

#endif /* COMMONS_FILTER_H */
