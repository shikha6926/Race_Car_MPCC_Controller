#ifndef SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_UTILS_DATA_CONVERSION
#define SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_UTILS_DATA_CONVERSION
#include <Eigen/Core>
#include <vector>

namespace commons
{
/**
 * @brief Converts a matrix to a vector of pointers so it can be used with casadi
 *
 * @tparam rows rows of the matrix
 * @tparam cols columns of the matrix
 * @param matrix the matrix
 * @return std::vector<double*>
 */
template <int rows, int cols>
std::vector<double*> convertToVector(Eigen::Matrix<double, rows, cols>& matrix)
{
  std::vector<double*> output(matrix.rows() * matrix.cols());
  for (int i = 0; i < matrix.rows(); i++)
  {
    for (int j = 0; j < matrix.cols(); j++)
    {
      output[i * matrix.cols() + j] = &matrix(i, j);
    }
  }
  return output;
}

/**
 * @brief Converts a state/input struct into a vector of pointers. (memory-less)
 * Note that this transformation is needed to be able to pass the struct to any casadi function
 *
 * @tparam Type Type of the state/input struct
 * @param state Struct that should be converted to a vector
 * @return std::vector<double*>  the vector representation
 */
template <typename Type>
std::vector<double*> convertToVector(Type& state);

/**
 * @brief Converts a state/input struct into a vector of const pointers. (memory-less)
 * Note that this transformation is needed to be able to pass the struct to any casadi function
 *
 * @tparam Type Type of the state/input struct
 * @param state Struct that should be converted to a vector
 * @return std::vector<double*>  the vector representation
 */
template <typename Type>
std::vector<const double*> convertToConstVector(const Type& state);

/**
 * @brief Converts an Eigen Matrix (vector) to a state struct
 *
 * @tparam StateType Type of the state struct that should be converted
 * @tparam StateDimension Dimension of the state. If unknown use -1
 * @param vector Eigen Matrix (vector) containing state information
 * @return StateType State Struct containing the data
 */
template <typename StateType, int StateDimension = -1>
StateType convertToState(const Eigen::Matrix<double, StateDimension, 1>& vector);

/**
 * @brief Converts a custom state struct to an Eigen Matrix (Vector)
 *
 * @tparam StateType State struct that should be converted
 * @tparam StateDimension Dimension of the state. If unknown use -1
 * @param state The state struct that should be converted
 * @return Eigen::Matrix<double, StateDimension, 1>  the resulting Matrix (Vector)
 */
template <typename StateType, int StateDimension = -1>
Eigen::Matrix<double, StateDimension, 1> convertToEigen(const StateType& state);

/**
 * @brief Converts a State and Input struct into a stacked vector of pointers.
 * The first elements will consist of the vector representation of the state, the second part of the input
 *
 * @tparam StateType type of the state struct
 * @tparam InputType type of the input struct
 * @param state State struct that should be converted
 * @param input Input struct that should be converted
 * @return std::vector<double*> vector containing the data of both state and input structs
 */
template <typename StateType, typename InputType>
std::vector<double*> convertToVector(StateType& state, InputType& input)
{
  std::vector<double*> state_as_vector = convertToVector(state);
  std::vector<double*> input_as_vector = convertToVector(input);

  std::vector<double*>& state_and_input_as_vector = state_as_vector;
  state_and_input_as_vector.insert(state_as_vector.end(), input_as_vector.begin(),
                                   input_as_vector.end());  // Append input at the end of state vector
  return state_and_input_as_vector;
}

/**
 * @brief Converts a State and Input struct into a stacked vector of const pointers.
 * The first elements will consist of the vector representation of the state, the second part of the input
 *
 * @tparam StateType type of the state struct
 * @tparam InputType type of the input struct
 * @param state State struct that should be converted
 * @param input Input struct that should be converted
 * @return std::vector<double*> vector containing the data of both state and input structs
 */
template <typename StateType, typename InputType>
std::vector<const double*> convertToConstVector(const StateType& state, const InputType& input)
{
  std::vector<const double*> state_as_vector = convertToConstVector(state);
  std::vector<const double*> input_as_vector = convertToConstVector(input);

  std::vector<const double*>& state_and_input_as_vector = state_as_vector;
  state_and_input_as_vector.insert(state_as_vector.end(), input_as_vector.begin(),
                                   input_as_vector.end());  // Append input at the end of state vector
  return state_and_input_as_vector;
}

}  // namespace commons
#endif /* SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_UTILS_DATA_CONVERSION */
