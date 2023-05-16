#include "commons/filter.h"

//!
/*!
 * implement digital Filter using difference equation in
 * direct II transposed structure form
 * \param sig new value of signal to Filter
 */
double Filter::process(const double& sig)
{
  double sig_filt = b_[0] * sig + z_[0];
  for (int i = 1; i != a_.size(); i++)
  {
    z_[i - 1] = b_[i] * sig + z_[i] - a_[i] * sig_filt;
  }
  return sig_filt;
};