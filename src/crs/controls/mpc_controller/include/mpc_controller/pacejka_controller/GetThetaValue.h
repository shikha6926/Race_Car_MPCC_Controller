#include <cmath>
#include <string>
#include <stdio.h>
#include <commons/static_track_trajectory.h>
#include <Eigen/Dense>

#include <mpc_controller/pacejka_controller/mpcc_pacejka_coef_copy.h>

extern std::vector<double> X_coef_0;
extern std::vector<double> X_coef_1;
extern std::vector<double> X_coef_2;
extern std::vector<double> X_coef_3;
extern std::vector<double> Y_coef_0;
extern std::vector<double> Y_coef_1;
extern std::vector<double> Y_coef_2;
extern std::vector<double> Y_coef_3;

inline double euclidean2Dist(double x1, double y1, double x2, double y2)
{
  return (std::sqrt(std::pow(x1 - x2, 2.0) + std::pow(y1 - y2, 2.0)));
}

extern std::vector<double> X_coords;
extern std::vector<double> Y_coords;
std::vector<double> rate_x;
std::vector<double> rate_y;
std::vector<double> curvature;           // NOLINT
std::vector<double> tangent_angle;
std::vector<double> arc_length;

crs_controls::StaticTrackTrajectory spline_track(X_coords, Y_coords, rate_x, rate_y, 0.5, 
                        curvature, tangent_angle, arc_length, 300);  

int getclosestsplineindex(Eigen::Vector2d state_pt)
{
    int spline_index;
    // Get the first Closest point id
    int first_id = spline_track.getClosestTrackPointIdx(state_pt);
    // Get the 2nd Closest point id
    // After getting the first id, select the spline
    int next_id = first_id + 1;
    int previous_id = first_id - 1;
    Eigen::Vector2d next_pt = spline_track.operator[](next_id);
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

double gettheta(int spline_index, Eigen::Vector2d state_pt)
{
    
    Eigen::Vector2d previous_pt = spline_track.operator[](spline_index);
    Eigen::Vector2d next_pt = spline_track.operator[](spline_index + 1);
    double spline_arclength = ((spline_index + 1) / 2.9367) - (spline_index / 2.9367);
    double spline_halved = spline_arclength / 2;
    double mid_theta = (spline_index / 2.9367) + spline_halved;
    double mid_X; 
    double mid_Y;
    double dist = 10;
    double next_point_dist, first_point_dist;
    double final_theta;
    double next_pt_theta = (spline_index / 2.9367) + spline_arclength;
    double previous_pt_theta = (spline_index / 2.9367);
    while (dist >= 0.2)
    {     
        first_point_dist = euclidean2Dist(previous_pt.x(), previous_pt.y(), state_pt.x(), state_pt.y());
        next_point_dist = euclidean2Dist(next_pt.x(), next_pt.y(), state_pt.x(), state_pt.y());
        spline_halved = spline_halved / 2;
        if(next_point_dist < first_point_dist)
        {    
            dist = next_point_dist;
            mid_X = X_coef_3[spline_index] +
                    X_coef_2[spline_index] * mid_theta + 
                    X_coef_1[spline_index] * std::pow(mid_theta, 2) +
                    X_coef_0[spline_index] * std::pow(mid_theta, 3);

            mid_Y = Y_coef_3[spline_index] +
                    Y_coef_2[spline_index] * mid_theta + 
                    Y_coef_1[spline_index] * std::pow(mid_theta, 2) +
                    Y_coef_0[spline_index] * std::pow(mid_theta, 3);
            previous_pt = Eigen::Vector2d(mid_X, mid_Y);            
            previous_pt_theta = mid_theta; 
            mid_theta = mid_theta + spline_halved;
        }
        else
        {
            dist = first_point_dist;
            mid_X = X_coef_3[spline_index] +
                    X_coef_2[spline_index] * mid_theta + 
                    X_coef_1[spline_index] * std::pow(mid_theta, 2) +
                    X_coef_0[spline_index] * std::pow(mid_theta, 3);

            mid_Y = Y_coef_3[spline_index] +
                    Y_coef_2[spline_index] * mid_theta + 
                    Y_coef_1[spline_index] * std::pow(mid_theta, 2) +
                    Y_coef_0[spline_index] * std::pow(mid_theta, 3);
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






