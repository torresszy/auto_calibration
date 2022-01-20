#ifndef calibration_hpp
#define calibration_hpp

#include "ceres/ceres.h"
#include "line_extractor.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

bool front_back_calibration;

void side_camera_calibration;

bool calibration;

void solve_one_frame_cpy;

bool solve_multi_frame_cpy;

void points_to_param;

double get_yaw;

void get_all_lines_from_pairs;

double c_py;

double c_rl;

double get_lane_width_from_params;

void solve_one_frame_crl;

bool solve_multi_frame_crl;

double get_lane_width;

double get_midpoint;

double c_sp;

void solve_one_side_csp;

void solve_one_frame_csp;

void solve_multi_frame_csp;

bool correspondency_check;

bool pre_lanemark_pairing_check;

void update_lanes;

#endif /* calibration_hpp */
