/**
 * @file    lighthouse_calibration_node.cpp
 * @author  Tobias Bodewig
 * @brief   Node that is used to calibrate the Lighthouse tracking system and determine the parameters for the Lighthouse sensor model.
 */

#include <math.h>
#include <ros/ros.h>
#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <ros_crs_utils/parameter_io.h>
#include "crs_msgs/lighthouse_sweep.h"
#include "ootx_decoder.h"

enum calibration_state 
{
  INIT,
  READ_CALIB_STREAM,
  INPUT_POINT_COUNT,
  INPUT_COORDINATES,
  COLLECT_DATA,
  FINISHED
};

/* Global variables --------------------------------------------------------- */
ootxDecoderState_t decoder_state_;

calibration_state state_ = INIT;

size_t current_point_;
size_t point_count_;

uint base_station_id_;

// Used to keep track of average during data collection phase
double angle_0_sum_;
double angle_1_sum_;
uint angle_0_count_;
uint angle_1_count_;

int sweeps_per_point_ = 50;
Eigen::Matrix<double, 2, 4> sensor_pos_ = Eigen::Matrix<double, 2, 4>::Zero();
Eigen::Matrix<double, 4, 4> R_ = Eigen::Matrix<double, 4, 4>::Identity();

std::vector<std::pair<double,double>> point_coordinates_;
std::vector<std::pair<double,double>> average_angles_;

/* Callback function -------------------------------------------------------- */
void lighthouseCallback(const crs_msgs::lighthouse_sweep& msg)
{
  switch (state_)
  {
  case INIT:
    {
      base_station_id_ = msg.polynomial >> 1;
      state_ = READ_CALIB_STREAM;
      std::cout << "Receiving calibration bit stream from base station. This can take up to a minute." << std::endl;
    }
    break;
  case READ_CALIB_STREAM:
    { 
      if (msg.first_sweep) 
      {
        bool is_complete = ootxDecoderProcessBit(&decoder_state_, msg.polynomial);
        // std::cout << "poly: " << unsigned(msg.polynomial) << " synced: " << decoder_state_.synchronized << std::endl;
        if (is_complete)
        {
          std::cout << "Received full calibration bit stream." << std::endl;
          std::cout << "tilt0: " << fp16_to_float(decoder_state_.frame.tilt0) << std::endl;
          std::cout << "tilt1: " << fp16_to_float(decoder_state_.frame.tilt1) << std::endl;
          state_ = INPUT_POINT_COUNT;
        }
      }
    }
    break;
  case COLLECT_DATA:
    {
      double average_angle = (msg.angle_0 + msg.angle_1 + msg.angle_2 + msg.angle_3) / 4;
      if (msg.first_sweep) {
        angle_0_sum_ += average_angle;
        angle_0_count_++;
      } else {
        angle_1_sum_ += average_angle;
        angle_1_count_++;
      }
      if (angle_0_count_ >= sweeps_per_point_ && angle_1_count_ >= sweeps_per_point_)
      {
        double average_angle_0 = angle_0_sum_ / angle_0_count_;
        double average_angle_1 = angle_1_sum_ / angle_1_count_;
        average_angles_.push_back(std::pair<double,double>(average_angle_0, average_angle_1));
        std::cout << "Angles recived: " << average_angle_0 << ", " << average_angle_1 << std::endl;
        current_point_++;
        if (current_point_ >= point_count_) {
          state_ = FINISHED;
        } else {
          state_ = INPUT_COORDINATES;
        }
      }
    }
    break;
  default:
    break;
  }
}

void outputMatrix(const Eigen::Matrix<double, -1,  -1> &m, std::ostream &out) 
{
  out << "[";
  for (size_t row = 0; row < m.rows(); row++)
  {
    out << "[";
    for (size_t col = 0; col < m.cols(); col++)
    {
      out << m(row, col);
      if (col < m.cols() - 1)
      {
        out << ", ";
      }
    }
    out << "]";
    if (row < m.rows() - 1)
    {
      out << ", ";
    }
  }
  out << "]";
}

/* Entry point -------------------------------------------------------------- */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_lighthouse_calibration");
  ros::NodeHandle nh;                                 // /<NAMESPACE>/*
  ros::NodeHandle nh_private = ros::NodeHandle("~");  // /<NAMESPACE>/lighthouse_calibration_node/*

  nh_private.getParam("sweeps_per_point", sweeps_per_point_);
  parameter_io::getMatrixFromParams<2, 4>(ros::NodeHandle(nh_private, "sensor_pos"), sensor_pos_);
  parameter_io::getMatrixFromParams<4, 4>(ros::NodeHandle(nh_private, "R"), R_);

  // Set up subscriber to /<NAMESPACE>/battery
  ros::Subscriber sub = nh.subscribe("lighthouse", 10, &lighthouseCallback);

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "Waiting for first Lighthouse msg." << std::endl;

  while(ros::ok() && state_ != FINISHED)
  {
    switch (state_)
    {
    case INPUT_POINT_COUNT:
      {
        std::cout << "Enter number of points that should be used for the calibration:" << std::endl;
        std::cin >> point_count_;
        if (point_count_ < 6)
        {
          std::cout << "A minimum of 6 points is required." << std::endl;
          break;
        }
        state_ = INPUT_COORDINATES;
      }
      break;
    case INPUT_COORDINATES:
      {
        double x, y;
        std::cout << "Position car on point number " << current_point_ + 1 << " of " << point_count_ << " on track." << std::endl;
        std::cout << "Enter x coordiante of point:" << std::endl;
        std::cin >> x;
        std::cout << "Enter y coordiante of point:" << std::endl;
        std::cin >> y;
        std::cout << "Collecting data for point " << current_point_ + 1 << ": x: " << x << " y: " << y << std::endl;
        point_coordinates_.push_back(std::pair<double,double>(x, y));
        angle_0_sum_ = 0;
        angle_1_sum_ = 0;
        angle_0_count_ = 0;
        angle_1_count_ = 0;
        state_ = COLLECT_DATA;
      }
      break;
    default:
      break;
    }
    ros::spinOnce();
  }

  double dt1 = fp16_to_float(decoder_state_.frame.tilt0);
  double dt2 = fp16_to_float(decoder_state_.frame.tilt1);
  double light_plane_tilt1 = -M_PI / 6 - dt1;
  double light_plane_tilt2 = M_PI / 6 - dt2;

  casadi::MX bs_angle_z = casadi::MX::sym("bs_angle_z");
  casadi::MX bs_angle_y = casadi::MX::sym("bs_angle_y");
  casadi::MX bs_angle_x = casadi::MX::sym("bs_angle_x");

  casadi::MX bs_pos_x = casadi::MX::sym("bs_pos_x");
  casadi::MX bs_pos_y = casadi::MX::sym("bs_pos_y");
  casadi::MX bs_pos_z = casadi::MX::sym("bs_pos_z");

  Eigen::Matrix<casadi::MX, 3, 3> rot_bsZ;
  Eigen::Matrix<casadi::MX, 3, 3> rot_bsY;
  Eigen::Matrix<casadi::MX, 3, 3> rot_bsX;
  rot_bsZ << cos(bs_angle_z), -sin(bs_angle_z), 0, sin(bs_angle_z), cos(bs_angle_z), 0, 0, 0, 1;
  rot_bsY << cos(bs_angle_y), 0, sin(bs_angle_y), 0, 1, 0, -sin(bs_angle_y), 0, cos(bs_angle_y);
  rot_bsX << 1, 0, 0, 0, cos(bs_angle_x), -sin(bs_angle_x), 0, sin(bs_angle_x), cos(bs_angle_x);
  Eigen::Matrix<casadi::MX, 3, 3> rot_bs = rot_bsZ * rot_bsY * rot_bsX;

  Eigen::Matrix<casadi::MX, 3, 1> pos_bs = {bs_pos_x, bs_pos_y, bs_pos_z};

  Eigen::Matrix<casadi::MX, 3, -1> calib_pos = Eigen::Matrix<casadi::MX, 3, -1>(3, point_count_);
  Eigen::Matrix<casadi::MX, 2, -1> calib_angle = Eigen::Matrix<casadi::MX, 2, -1>(2, point_count_);
  for (size_t i = 0; i < point_count_; i++)
  {
    calib_pos(0, i) = point_coordinates_[i].first;
    calib_pos(1, i) = point_coordinates_[i].second;
    calib_pos(2, i) = 0;
    calib_angle(0, i) = average_angles_[i].first;
    calib_angle(1, i) = average_angles_[i].second;
  }

  // Positions in base satation reference frame
  Eigen::Matrix<casadi::MX, 3, -1> sbs = rot_bs.transpose() * (calib_pos.colwise() - pos_bs);
  // Calculate angles
  Eigen::Matrix<casadi::MX, 1, -1> alphas = (sbs.row(1).array() / sbs.row(0).array()).atan();
  Eigen::Matrix<casadi::MX, 1, -1> alpha1 = alphas.array() + ((sbs.row(2) * tan(light_plane_tilt1)).array() / 
                                          (sbs.row(0).array().square() + sbs.row(1).array().square()).sqrt()).asin();
  Eigen::Matrix<casadi::MX, 1, -1> alpha2 = alphas.array() + ((sbs.row(2) * tan(light_plane_tilt2)).array() / 
                                          (sbs.row(0).array().square() + sbs.row(1).array().square()).sqrt()).asin();

  // Optimization function: minimize error in angles
  casadi::MX f = 0;
  for (size_t i = 0; i < point_count_; i++)
  {
    casadi::MX error1 = alpha1(i) - calib_angle(0, i);
    casadi::MX error2 = alpha2(i) - calib_angle(1, i);
    f += error1 * error1 + error2 * error2;
  }
  std::vector<casadi::MX> variables = {bs_angle_z, bs_angle_y, bs_angle_x, bs_pos_x, bs_pos_y, bs_pos_z};

  // Solver
  casadi::MXDict nlp =  {{"x", casadi::MX::vertcat(variables)}, {"f", f}};
  casadi::Function solver = casadi::nlpsol("solver", "ipopt", nlp);

  casadi::DMDict arg;
  arg["x0"] = std::vector<double>{0, M_PI / 3, M_PI / 3, 0, 0, 1};
  casadi::DMDict res = solver(arg);

  //  Print solution
  std::vector<double> solution = std::vector<double>(res.at("x"));

  Eigen::Matrix3d rot_solutionZ;
  Eigen::Matrix3d rot_solutionY;
  Eigen::Matrix3d rot_solutionX;
  rot_solutionZ << cos(solution[0]), -sin(solution[0]), 0, sin(solution[0]), cos(solution[0]), 0, 0, 0, 1;
  rot_solutionY << cos(solution[1]), 0, sin(solution[1]), 0, 1, 0, -sin(solution[1]), 0, cos(solution[1]);
  rot_solutionX << 1, 0, 0, 0, cos(solution[2]), -sin(solution[2]), 0, sin(solution[2]), cos(solution[2]);
  Eigen::Matrix3d rot_solution = rot_solutionZ * rot_solutionY * rot_solutionX;

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "###################################################" << std::endl;
  std::cout << "################ Calibration data #################" << std::endl;
  std::cout << "###################################################" << std::endl;
  std::cout << std::endl;
  std::cout << "Points: ";
  outputMatrix(calib_pos.cast<double>(), std::cout);
  std::cout << std::endl;
  std::cout << "Angles: ";
  outputMatrix(calib_angle.cast<double>(), std::cout);
  std::cout << std::endl;


  std::cout << std::endl;
  std::cout << "###################################################" << std::endl;
  std::cout << "################ Sensor model data ################" << std::endl;
  std::cout << "###################################################" << std::endl;
  std::cout << std::endl;
  std::cout << "  lighthouse:" << std::endl;
  std::cout << "    sensor_pos:" << std::endl;
  std::cout << "      value: ";
  outputMatrix(sensor_pos_, std::cout);
  std::cout << std::endl;
  std::cout << "    key: lighthouse" << std::endl;
  std::cout << "    base_stations: [\"bs" << base_station_id_ << "\"]" << std::endl;
  std::cout << "    bs" << base_station_id_ << ":" << std::endl;
  std::cout << "      bs_ID: " << base_station_id_ << std::endl;
  std::cout << "      R:" << std::endl;
  std::cout << "        value: ";
  outputMatrix(R_, std::cout);
  std::cout << std::endl;
  std::cout << "      P_bs:" << std::endl;
  std::cout << "        value: [[" << solution[3] << "], [" << solution[4] << "], [" << solution[5] << "]]" << std::endl;
  std::cout << "      R_bs:" << std::endl;
  std::cout << "        value: ";
  outputMatrix(rot_solution, std::cout);
  std::cout << std::endl;
  std::cout << "      dt1: " << dt1 << std::endl;
  std::cout << "      dt2: " << dt2 << std::endl;
  std::cout << std::endl;
  std::cout << "###################################################" << std::endl;
  std::cout << "###################################################" << std::endl;
  std::cout << "###################################################" << std::endl;
  std::cout << std::endl;

  std::cout << "Calibration finished!" << std::endl;

  return 0;
}

