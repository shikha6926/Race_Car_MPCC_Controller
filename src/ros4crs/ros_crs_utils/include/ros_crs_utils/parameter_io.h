#ifndef ROS_CRS_UTILS_PARAMETER_IO_H
#define ROS_CRS_UTILS_PARAMETER_IO_H

#include <ros/ros.h>
#include <memory>
#include <commons/static_track_trajectory.h>

#include <Eigen/Core>

namespace parameter_io
{

/**
 * @brief Loads a track manager based on the track description of the ros parameter server
 *
 * @param nh
 * @return std::shared_ptr<crs_controls::StaticTrackTrajectory>
 */
std::shared_ptr<crs_controls::StaticTrackTrajectory> loadTrackDescriptionFromParams(const ros::NodeHandle& nh);

/**
 * @brief Loads the parameters for a given model from the ros parameters server
 *
 * @tparam ParamsType type of parameters to load (e.g. pacejka_params)
 * @param nh Nodehandle to load the parameters
 * @param params The parameter object that should be populate
 * @param verbose if false, does not print warnings if parameters is not found
 */
template <typename ParamsType>
void getModelParams(const ros::NodeHandle& nh, ParamsType& params, bool verbose = true);

/**
 * @brief Get the state that is stored on the paremeter server
 *
 * @tparam StateType The State Type (e.g. pacejka_car_state)
 * @param nh NodeHandle
 * @return StateType The state stored on the parameter server
 */
template <typename StateType>
StateType getState(const ros::NodeHandle& nh);

/**
 * @brief Get the input that is stored on the paremeter server
 *
 * @tparam InputType The Input Type (e.g. pacejka_car_input)
 * @param nh NodeHandle
 * @return InputType The state stored on the parameter server
 */
template <typename InputType>
InputType getInput(const ros::NodeHandle& nh);

/**
 * @brief Loads the given ConfigType struct from ros params
 *
 * @tparam ConfigType the object that should be loaded
 * @param nh NodeHandle to load the parameters
 */
template <typename ConfigType>
ConfigType getConfig(const ros::NodeHandle& nh);

/**
 * @brief Loads a matrix from rosparams
 *
 * @tparam Rows Number of rows of the matrix
 * @tparam Cols Number of columns of the matrix
 * @param nh Nodehandle pointing to the parameters
 * @param matrix The matrix that should be populated
 * @return true if matrix could be loaded
 */
template <int Rows, int Cols>
bool getMatrixFromParams(const ros::NodeHandle& nh, Eigen::Matrix<double, Rows, Cols>& matrix)
{
  XmlRpc::XmlRpcValue matrix_as_vector;
  if (!nh.getParam("value", matrix_as_vector))
    return false;

  bool is_diagonal = false;
  nh.getParam("is_diag", is_diagonal);

  int n_rows = matrix_as_vector.size();

  if (n_rows != Rows)
    return false;

  assert(n_rows > 0);

  if (is_diagonal)
  {
    matrix.resize(n_rows, n_rows);
    matrix.setZero();
    for (int i = 0; i < n_rows; i++)
    {
      auto value = matrix_as_vector[i][0];
      if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
        matrix(i, i) = static_cast<double>((int)value);
      else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        matrix(i, i) = static_cast<double>(value);
    }

    return true;
  }

  // E.g [[0,1,2], [0,1,2]] -> 2 rows, 3 cols
  int n_cols = matrix_as_vector[0].size();
  if (n_cols != Cols)
    return false;
  matrix.resize(n_rows, n_cols);
  matrix.setZero(n_rows, n_cols);

  for (int i = 0; i < n_rows; i++)
  {
    for (int k = 0; k < n_cols; k++)
    {
      auto value = matrix_as_vector[i][k];

      if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
        matrix(i, k) = static_cast<double>((int)value);
      else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        matrix(i, k) = static_cast<double>(value);
    }
  }

  return true;
}

}  // namespace parameter_io
#endif /* ROS_CRS_UTILS_PARAMETER_IO_H */
