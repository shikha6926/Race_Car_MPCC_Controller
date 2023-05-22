#include "commons/trajectory_spline.h"
#include <iostream>
#include <numeric>
#include <Eigen/Core>
#include <vector>

namespace crs_controls
{
//     StaticTrackTrajectory::StaticTrackTrajectory(const std::vector<Eigen::Vector2d>& points) : Trajectory()
// {
//   x_coords_.reserve(points.size());
//   y_coords_.reserve(points.size());
//   for (const auto& pt : points)
//   {
//     x_coords_.push_back(pt.x());
//     y_coords_.push_back(pt.y());
//   }
// }
const double SplineTrajectory::getLastRequestedTrackAngle() const
{
  return 1.0;
}

inline double SplineTrajectory::euclidean2Dist(double x1, double y1, double x2, double y2)
{
  return (std::sqrt(std::pow(x1 - x2, 2.0) + std::pow(y1 - y2, 2.0)));
}

int SplineTrajectory::getclosestsplineindex(Eigen::Vector2d state_pt, SplineTrajectory spline_track)
{  
    int spline_index;
    // Get the first Closest point id
    int first_id = spline_track.getClosestTrackPointIdx(state_pt);
    // Get the 2nd Closest point id
    // After getting the first id, select the spline
    int next_id = first_id + 1;
    int previous_id = first_id - 1;
    Eigen::Vector2d next_pt =spline_track.operator[](next_id);
    Eigen::Vector2d previous_pt = spline_track.operator[](previous_id);
    
    if (euclidean2Dist(next_pt.x(), next_pt.y(), state_pt.x(), state_pt.y()) > 
    euclidean2Dist(previous_pt.x(), previous_pt.y(), state_pt.x(), state_pt.y()))
    {
        spline_index = previous_id;
    }
    else{
        spline_index = first_id;
    }

    return spline_index;
}

double SplineTrajectory::gettheta(int spline_index, Eigen::Vector2d state_pt, SplineTrajectory spline_track)
{
    double density =  2.9367;
    Eigen::Vector2d previous_pt = spline_track.operator[](spline_index);
    Eigen::Vector2d next_pt = spline_track.operator[](spline_index + 1);
    double spline_arclength = ((spline_index + 1) /density) - (spline_index /density);
    double spline_halved = spline_arclength / 2;
    double mid_theta = (spline_index /density) + spline_halved;
    double mid_X; 
    double mid_Y;
    double dist = 10;
    double next_point_dist, first_point_dist;
    double final_theta;
    double next_pt_theta = (spline_index / density) + spline_arclength;
    double previous_pt_theta = (spline_index / density);
    while (dist >= 0.2)
    {     
        first_point_dist = euclidean2Dist(previous_pt.x(), previous_pt.y(), state_pt.x(), state_pt.y());
        next_point_dist = euclidean2Dist(next_pt.x(), next_pt.y(), state_pt.x(), state_pt.y());
        spline_halved = spline_halved / 2;
        if(next_point_dist < first_point_dist)
        {    
            dist = next_point_dist;
            mid_X = X_coef_3_[spline_index] +
                    X_coef_2_[spline_index] * mid_theta + 
                    X_coef_1_[spline_index] * std::pow(mid_theta, 2) +
                    X_coef_0_[spline_index] * std::pow(mid_theta, 3);

            mid_Y = Y_coef_3_[spline_index] +
                    Y_coef_2_[spline_index] * mid_theta + 
                    Y_coef_1_[spline_index] * std::pow(mid_theta, 2) +
                    Y_coef_0_[spline_index] * std::pow(mid_theta, 3);
            previous_pt = Eigen::Vector2d(mid_X, mid_Y);            
            previous_pt_theta = mid_theta; 
            mid_theta = mid_theta + spline_halved;
        }
        else
        {
            dist = first_point_dist;
            mid_X = X_coef_3_[spline_index] +
                    X_coef_2_[spline_index] * mid_theta + 
                    X_coef_1_[spline_index] * std::pow(mid_theta, 2) +
                    X_coef_0_[spline_index] * std::pow(mid_theta, 3);

            mid_Y = Y_coef_3_[spline_index] +
                    Y_coef_2_[spline_index] * mid_theta + 
                    Y_coef_1_[spline_index] * std::pow(mid_theta, 2) +
                    Y_coef_0_[spline_index] * std::pow(mid_theta, 3);
            next_pt = Eigen::Vector2d(mid_X, mid_Y);
            next_pt_theta = mid_theta;
            mid_theta = mid_theta - spline_halved;
        }

    }
    first_point_dist = euclidean2Dist(previous_pt.x(), previous_pt.y(), state_pt.x(), state_pt.y());
    next_point_dist = euclidean2Dist(next_pt.x(), next_pt.y(), state_pt.x(), state_pt.y());
    if (next_point_dist < first_point_dist)
    {
        final_theta = next_pt_theta;
    }
    else{
        final_theta = previous_pt_theta;
    }
    return final_theta;
}

Eigen::Vector2d SplineTrajectory::getRefCoords(double distance_on_track) 
{
    int idx = distance_on_track * SplineTrajectory::getDensity() - 1;
    double distance = distance_on_track - ((double)idx / SplineTrajectory::getDensity());
    std::cout<< "distance "<< distance << "\n";
    std::cout<< "idx "<< idx << "\n";
    double x_coords_= X_coef_3_[idx] + 
            X_coef_2_[idx] * distance + 
            X_coef_1_[idx] * std::pow(distance, 2) + 
            X_coef_0_[idx] * std::pow(distance, 3);

    double y_coords_ = Y_coef_3_[idx] +
                 Y_coef_2_[idx] * distance + 
                 Y_coef_1_[idx] * std::pow(distance, 2) +
                 Y_coef_0_[idx] * std::pow(distance, 3);
    return Eigen::Vector2d(x_coords_, y_coords_);
}

Eigen::Vector2d SplineTrajectory::getGradient(double distance_on_track)
{
    int idx = distance_on_track * SplineTrajectory::getDensity() - 1;
    double distance = distance_on_track - ((double)idx / SplineTrajectory::getDensity());
    double x_rate_= X_coef_2_[idx] +
            2 * X_coef_1_[idx] * distance + 
            3 * X_coef_0_[idx] * std::pow(distance, 2); 

    double y_rate_ = Y_coef_2_[idx] +
            2 * Y_coef_1_[idx] * distance + 
            3 * Y_coef_0_[idx] * std::pow(distance, 2);    
    return Eigen::Vector2d(x_rate_, y_rate_);
}

double SplineTrajectory::getTangentAngle(double y_rate , double x_rate)
{
    double phi_ = atan2(y_rate , x_rate);
    return phi_;

}   
 
}

