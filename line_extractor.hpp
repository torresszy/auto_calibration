#ifndef line_extractor_hpp
#define line_extractor_hpp

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>
#include <opencv2/ximgproc/edge_drawing.hpp>
#include <iostream>
#include <stdio.h>
#include "glog/logging.h"

using namespace cv;
using namespace std;

/* Struct holding the parameters for one camera */
struct CalibParams {
    string name;
    double fu;
    double fv;
    double cu;
    double cv;
    double dist_coeffs_0;
    double dist_coeffs_1;
    double dist_coeffs_2;
    double dist_coeffs_3;
    double camera_x;
    double camera_y;
    double camera_z;
    double pitch;
    double yaw;
    double roll;
};


/* Struct for all four cameras */
struct camera_set {
    CalibParams* front;
    CalibParams* rear;
    CalibParams* left;
    CalibParams* right;
};


/* Struct for lanemarks */
struct lanemarks {
    Vec4f rising_edge;
    Vec4f falling_edge;
};


struct one_frame_lines {
    vector<lanemarks> front_lanes;
    vector<lanemarks> front_stop_lines;
    vector<lanemarks> rear_lanes;
    vector<lanemarks> rear_stop_lines;
    vector<lanemarks> left_lanes;
    vector<lanemarks> right_lanes;
    uint64_t timestamp;
};


struct one_frame_lines_set {
    one_frame_lines* orig_lines;
    one_frame_lines* pre_filtered_lines;
    one_frame_lines* filtered_lines;
    one_frame_lines* front_back_cpy_lines;
    one_frame_lines* front_back_crl_lines;
    one_frame_lines* side_cpy_lines;
    one_frame_lines* side_csp_lines;
    one_frame_lines* final_lines;
};

struct vanishing_pts {
    Point2f front_vp;
    Point2f rear_vp;
    Point2f left_vp;
    Point2f right_vp;
};


/* create a new CalibParams struct and initialize it
 * Parameters see definition for CalibParams
 */
CalibParams* camera_new(string name, double fu, double fv, double cu, double cv,
                        double dis_coeff_0, double dis_coeff_1,
                        double dis_coeff_2, double dis_coeff_3,
                        double camera_x, double camera_y, double camera_z);

one_frame_lines_set* one_frame_set_new();

/* Initialize the intrinsic parameters based on a camera
 *
 * camera: a pointer to CalibParams holding the parameters of the camera
 * cam_matrix: a pointer holding the return val&ue of the camera matrix
 * dist_coeffs: a pointer holding the return value of the distortion coefficients
 */
void init_intrinsic(CalibParams* camera, Mat* cam_matrix, Mat* dist_coeffs);


/* Calculate the rotation matrix R based on a camera's pitch, yaw, and roll angles
 *
 * camera: a pointer to CalibParams holding the parameters of the camera
 * R: a pointer holding the return value of the rotation matrix
 */
void rotation_homography(CalibParams* camera, Mat* R);


/* Calculate the translation matrix R based on the rotation matrix
 * ******rotation_homography MUST BE CALLED FIRST******
 *
 * camera: a pointer to CalibParams holding the parameters of the camera
 * R: a pointer holding the return value of the rotation matrix
 * T: a pointer holding the return value of the translation matrix
 */
void translation_homography(CalibParams* camera, Mat* R, Mat* T);

void ENURotationFromEuler(double roll_degree, double pitch_degree, double yaw_degree, cv::Mat* rot_mat);

/* Adjust the orientation of the rotation and translation matrix from zyx to yxz
 *
 * R: a pointer to the rotation matrix
 * T: a pointer to the translation matrix
 */
void homography_adjust(Mat* R, Mat* T);


//RANSAC fit 2D straight line
//Input parameters: points--input point set
// iterations--number of iterations
// sigma--The acceptable difference between the data and the model, the lane line pixel bandwidth is generally about 10
//              （Parameter use to compute the fitting score）
// k_min/k_max--The value range of the slope of the fitted straight line.
// Considering that the slope of the left and right lane lines in the image is within a certain range,
// Adding this parameter can also avoid detecting vertical and horizontal lines
//Output parameters: line--fitted line parameters, It is a vector of 4 floats
//              (vx, vy, x0, y0) where (vx, vy) is a normalized
//              vector collinear to the line and (x0, y0) is some
//              point on the line.
//Return value: none
//void fitLineRansac(const std::vector<cv::Point2f>& points,
//                   cv::Vec4f &line,
//                   int iterations = 1000,
//                   double sigma = 1.,
//                   double k_min = -7.,
//                   double k_max = 7.);


/* Detects the lanemark lines in the birdeye image
 *
 * birdeye_img: a pointer to the birdeye image
 */
void detect_line(Mat* birdeye_img, vector<Vec4f>* lines, vector<lanemarks>* pairs);


/* transorm one fisheye image into a birdeye image with the camera parameters
 *
 * camera: a pointer to CalibParams holding the parameters of the camera
 * img: a pointer to the fisheye image
 */
void birdeye_oneview_transform(Mat* img, Mat* birdeye_oneview, CalibParams* camera, Point2f* lu, Point2f* rb);


void oneview_extract_line(Mat* img, Mat* birdeye_img, CalibParams* camera, one_frame_lines_set* res, vanishing_pts* v_pts);

/* transform all four fisheye images with four cameras' parameters
 *
 * cameras: a pointer to camera_set struct holding parameters of all four cameras
 * all_img: a pointer to all four fisheye images
 */
void birdeye_transform(Mat* all_img, camera_set* cameras, vector<one_frame_lines_set>* multi_frame_set, vector<vanishing_pts>* vanishing_pts_set);


void line_pairing(vector<Vec4f>* lines, vector<lanemarks>* pairs);

//RANSAC fit 2D straight line
//Input parameters: points--input point set
// iterations--number of iterations
// sigma--The acceptable difference between the data and the model, the lane line pixel bandwidth is generally about 10
//              （Parameter use to compute the fitting score）
// k_min/k_max--The value range of the slope of the fitted straight line.
// Considering that the slope of the left and right lane lines in the image is within a certain range,
// Adding this parameter can also avoid detecting vertical and horizontal lines
//Output parameters: line--fitted line parameters, It is a vector of 4 floats
//              (vx, vy, x0, y0) where (vx, vy) is a normalized
//              vector collinear to the line and (x0, y0) is some
//              point on the line.
//Return value: none
void fitLineRansac(const std::vector<cv::Point2f>& points,
                  cv::Vec4f &line,
                  int iterations = 1000,
                  double sigma = 1.,
                  double k_min = -7.,
                   double k_max = 7.);


static float GetDist(float x1, float y1, float x2, float y2);

// get angle with positive vale between 2 line segments
static float GetAngle(float x1, float y1, float x2, float y2);

// get angle with positive value between input line and iamge x axis
static float GetAngle(Vec4f& line);

static float GetX(Vec4f& line, float y);

// determine whether 2 lines is colinear
static float GetSimilarity(Vec4f& l1, Vec4f& l2);

static void Kmeans(const std::vector<std::pair<int, float>>& sample, int k,
                   std::vector<std::vector<std::pair<int, float>>>* res);

void FilterLines(vector<lanemarks>& lines, CalibParams* camera, one_frame_lines* res, one_frame_lines* pre_filtered);

void duplicate_filter(vector<Vec4f>* lines);

void length_filter(vector<Vec4f>* lines);

void range_filter(vector<Vec4f>* lines);

void orientation_filter(vector<Vec4f>* lines);

void average_vanish_pt(vector<Point2f>* v_pts, Point2f* averaged_pt);

void get_vanish_pt(CalibParams* camera, Point2f* vanishing_pt, one_frame_lines* res);

double get_pitch(CalibParams* camera, Point2f* vanishing_pt);

void get_Hmax(CalibParams* camera, one_frame_lines* res, Mat* Hmax, vanishing_pts* v_pts);

void side_cam_Hmax(CalibParams* camera, one_frame_lines* res, Mat* Hmax, vanishing_pts* v_pts);

void front_back_cam_Hmax(CalibParams* camera, one_frame_lines* res, Mat* Hmax, vanishing_pts* v_pts);

void get_h(Point2f* vanish_pt, Mat* H);

void get_h(Mat* R, Mat* H);

void get_h(double vx, double vy, const double vz, Mat* H);

void calculate_H(Mat* R, Mat* K, Mat* H);

void get_virtual_K(Mat* K);

void get_updated_R(Mat* u, double theta, Mat* R);

double get_rotation_axis_and_angle(Mat* u, Mat* vanish_pt);

void outlier_filter(CalibParams* camera, one_frame_lines* tmp_res, Mat* Hmax, one_frame_lines* new_res);

float get_slope(float x1, float y1, float x2, float y2);

float get_intercept(float y, float x, float m);

float get_x(float y, float y_coeff, float m, float b);

bool location_check(lanemarks* lane, const double center_axis);

void find_closest_lane(vector<lanemarks>* lanes, bool left_side, lanemarks* cloest_lane);

bool cluter_filter(Point2f p1, Point2f p2);

void get_cluster_median(vector<int>* cluster_indices, int cluster_num, vector<Point2f>* pts, Point2f* final_pt);

void cluster_vpt_set(vector<vanishing_pts>* vanish_pts, vanishing_pts* final_v_pts);

void lanes_filter(CalibParams* camera, one_frame_lines* res, one_frame_lines* pre_filtered, double center_axis);

#endif /* line_extractor_hpp */

