#ifndef calibration_hpp
#define calibration_hpp

#include "ceres/ceres.h"
#include "line_extractor.hpp"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

bool front_back_calibration(double frame_num, CalibParams* camera, vector<one_frame_lines_set>* res, Point2f* vp, const double h0, const double hmax);

void side_camera_calibration(double frame_num, CalibParams* camera, vector<one_frame_lines_set>* res, Point2f* vp,
                             const double h0, const double hmax, const double center_axis);

bool calibration(double frame_num, camera_set* cameras, vector<one_frame_lines_set>* res, vanishing_pts* v_pts);

void solve_one_frame_cpy(Problem* problem, double* v, vector<lanemarks>* lanes, const double h0, const double hmax);

bool solve_multi_frame_cpy(int frame_num, double* v, CalibParams* camera, vector<one_frame_lines_set>* multi_frame_set,
                           const double h0, const double hmax);

void points_to_param(double* param, Vec4f points);

double get_yaw(CalibParams* camera, Point2f* vanishing_pt);

void get_all_lines_from_pairs(vector<lanemarks>* lanes, vector<Vec4f>* lines);

double c_py(double vx, double vy, double* l1,
            double* l2, const double h0, const double hmax);

double c_rl(double pitch, double yaw, double roll, double* l1_l, double* l1_r, double* l2_l, double* l2_r);

double get_lane_width_from_params(double* l1, double* l2);

void solve_one_frame_crl(Problem* problem, vector<lanemarks>* lanes, const double pitch, const double yaw, double* roll);

bool solve_multi_frame_crl(int frame_num, CalibParams* camera, vector<one_frame_lines_set>* multi_frame_set, double* roll);

double get_lane_width(Vec4f rising_edge, Vec4f falling_edge);

double get_midpoint(double h, double* l1, double* l2);

double c_sp(double h, double pitch, double yaw, double roll, double* side_l1, double* side_l2, double* front_back_l1, double* front_back_l2);

void solve_one_side_csp(Problem* problem, lanemarks* side_line, lanemarks* front_back_line, const double roll, const double yaw, double h, double* pitch, vector<double*>* line_params);

void solve_one_frame_csp(CalibParams* camera, Problem* problem, one_frame_lines* lanes,
                         const double roll, const double yaw, const double center_axis,
                         double front_h, double rear_h, int* cnt, double* pitch, vector<double*>* line_params);

void solve_multi_frame_csp(CalibParams* camera, vector<one_frame_lines_set>* res, const double center_axis,
                           double front_h, double rear_h, double frame_num, double* pitch);

bool correspondency_check(lanemarks* side_lane, lanemarks* front_back_lane, double threshold, double h);

bool pre_lanemark_pairing_check(vector<lanemarks>* lanes, double center_axis);

void update_lanes(vector<lanemarks>* lanes, Mat* H);

#endif /* calibration_hpp */
