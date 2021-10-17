#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "calibration.hpp"

using namespace cv;
using namespace std;

void birdeye_transform(Mat* all_img, camera_set* cameras, vector<one_frame_lines_set>* multi_frame_set, vector<vanishing_pts>* vanishing_pts_set)
{
    Mat birdeye_img(Size(352, 704), CV_8UC3); //numbers from avm_w and avm_h
    Mat img_crop;
    one_frame_lines_set* res = one_frame_set_new();
    vanishing_pts* v_pts = new vanishing_pts;
    
    for (int i = 0; i < 4; i++) {
        if (i == 0) {
            Rect roi(0, 0, 1280, 720);
            resize((*all_img)(roi), img_crop, Size(1344, 968));
            oneview_extract_line(&img_crop, &birdeye_img, cameras->front, res, v_pts);
        } else if (i == 1) {
            Rect roi(1280, 0, 1280, 720);
            resize((*all_img)(roi), img_crop, Size(1344, 968));
            oneview_extract_line(&img_crop, &birdeye_img, cameras->rear, res, v_pts);
        } else if (i == 2) {
            Rect roi(0, 720, 1280, 720);
            resize((*all_img)(roi), img_crop, Size(1344, 968));
            oneview_extract_line(&img_crop, &birdeye_img, cameras->left, res, v_pts);
        } else if (i == 3) {
            Rect roi(1280, 720, 1280, 720);
            resize((*all_img)(roi), img_crop, Size(1344, 968));
            oneview_extract_line(&img_crop, &birdeye_img, cameras->right, res, v_pts);
        }
    }

    multi_frame_set->push_back(*res);
    vanishing_pts_set->push_back(*v_pts);
    
//
//    std::vector<lanemarks> lines_orig;
//    lines_orig.insert(lines_orig.end(), res->orig_lines->front_lanes.begin(), res->orig_lines->front_lanes.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->front_stop_lines.begin(), res->orig_lines->front_stop_lines.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->rear_lanes.begin(), res->orig_lines->rear_lanes.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->rear_stop_lines.begin(), res->orig_lines->rear_stop_lines.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->left_lanes.begin(), res->orig_lines->left_lanes.end());
//    lines_orig.insert(lines_orig.end(), res->orig_lines->right_lanes.begin(), res->orig_lines->right_lanes.end());
//
//    Mat img_orig = birdeye_img.clone();
//    for (uint32_t i = 0; i < lines_orig.size(); i++) {
//        cv::Vec4f line = lines_orig[i].rising_edge;
//        cv::circle(img_orig, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_orig, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_orig, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255, 0, 0), 1);
//        line = lines_orig[i].falling_edge;
//        cv::circle(img_orig, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_orig, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_orig, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(0, 255, 0), 1);
//    }
//    cv::imshow("original", birdeye_img);
//
//    std::vector<lanemarks> lines_pre_filtered;
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->front_lanes.begin(), res->pre_filtered_lines->front_lanes.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->front_stop_lines.begin(), res->pre_filtered_lines->front_stop_lines.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->rear_lanes.begin(), res->pre_filtered_lines->rear_lanes.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->rear_stop_lines.begin(), res->pre_filtered_lines->rear_stop_lines.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->left_lanes.begin(), res->pre_filtered_lines->left_lanes.end());
//    lines_pre_filtered.insert(lines_pre_filtered.end(), res->pre_filtered_lines->right_lanes.begin(), res->pre_filtered_lines->right_lanes.end());
//
//    Mat img_pre_filtered = birdeye_img.clone();
//    for (uint32_t i = 0; i < lines_pre_filtered.size(); i++) {
//        cv::Vec4f line = lines_pre_filtered[i].rising_edge;
//        cv::circle(img_pre_filtered, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_pre_filtered, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_pre_filtered, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255, 0, 0), 1);
//        line = lines_pre_filtered[i].falling_edge;
//        cv::circle(img_pre_filtered, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_pre_filtered, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_pre_filtered, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(0, 255, 0), 1);
//    }
//    cv::imshow("pre_filtered", img_pre_filtered);
    
//
//    std::vector<lanemarks> lines_final;
//    lines_final.insert(lines_final.end(), res->filtered_lines->front_lanes.begin(), res->filtered_lines->front_lanes.end());
////    lines_final.insert(lines_final.end(), res->filtered_lines->front_stop_lines.begin(), res->filtered_lines->front_stop_lines.end());
//    lines_final.insert(lines_final.end(), res->filtered_lines->rear_lanes.begin(), res->filtered_lines->rear_lanes.end());
////    lines_final.insert(lines_final.end(), res->filtered_lines->rear_stop_lines.begin(), res->filtered_lines->rear_stop_lines.end());
////    lines_final.insert(lines_final.end(), res->filtered_lines->left_lanes.begin(), res->filtered_lines->left_lanes.end());
////    lines_final.insert(lines_final.end(), res->filtered_lines->right_lanes.begin(), res->filtered_lines->right_lanes.end());
//
//    Mat img_final = birdeye_img.clone();
//    for (uint32_t i = 0; i < lines_final.size(); i++) {
//        cv::Vec4f line = lines_final[i].rising_edge;
//        cv::circle(img_final, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_final, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_final, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255, 0, 0), 1);
//        line = lines_final[i].falling_edge;
//        cv::circle(img_final, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::circle(img_final, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255, 0, 255), 1);
//        cv::line(img_final, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(0, 255, 0), 1);
//    }
//    cv::imshow("line extractor res", img_final);

}

int main() {
    
    // read file
    string path = "Resources/fisheye.MP4";
    VideoCapture cap(path);
    Mat img, img_crop;
    
    // input parameters for the camera set
    camera_set* cameras = new camera_set;
    camera_set* cameras_v = new camera_set;
    
    // input parameters for each camera
    CalibParams* front = camera_new("front", 342, 339, 669, 471, 7.4000818911000327e-02, -1.4119762800860764e-02, -3.8102145102513209e-03, 3.3996593689209761e-04, 2.414, 0, -0.735);
    front->roll = 0;
    front->pitch = -16;
    front->yaw = 0;
    
    CalibParams* right = camera_new("right", 332, 320, 671, 489, 9.7534216107128202e-02, -1.1684532346081340e-03, -1.3137452012102382e-02, 2.2573161333372619e-03, 0.701, 0.907, -0.902);
    right->roll = 0;
    right->pitch = -20;
    right->yaw = 95;
    
    CalibParams* left = camera_new("left", 333, 331, 662, 485, 1.2535170775731322e-01, -6.3233407859422594e-02, 2.4865957700149491e-02, -5.8029832006416607e-03, 0.732, -0.907, -0.920);
    left->roll = 0;
    left->pitch = -15;
    left->yaw = -100;
    
    CalibParams* rear = camera_new("rear", 342, 339, 669, 471, 7.4000818911000327e-02, -1.4119762800860764e-02, -3.8102145102513209e-03, 3.3996593689209761e-04, -2.414, 0, -0.675);
    rear->roll = 0;
    rear->pitch = -11;
    rear->yaw = 180;

    CalibParams* front_v = camera_new("front", 45.04, 37.39, 176, 352, 0, 0, 0, 0, 0, 0, 0);
    CalibParams* rear_v = camera_new("rear", 45.04, 37.39, 176, 352, 0, 0, 0, 0, 0, 0, 0);
    CalibParams* left_v = camera_new("left", 45.04, 37.39, 176, 352, 0, 0, 0, 0, 0, 0, 0);
    CalibParams* right_v = camera_new("right", 45.04, 37.39, 176, 352, 0, 0, 0, 0, 0, 0, 0);
    
    cameras->front = front;
    cameras->rear = rear;
    cameras->left = left;
    cameras->right = right;
    
    cameras_v->front = front_v;
    cameras_v->rear = rear_v;
    cameras_v->left = left_v;
    cameras_v->right = right_v;
    
    cap.set(CAP_PROP_POS_FRAMES, 230);
    int frame_num = 50;
    
    while (true) {
        vector<one_frame_lines_set> multi_frame_set;
        vector<vanishing_pts> vanishing_pts_set;
        for (int i = 0; i < frame_num; i ++) {
            cout << "i = " << i << endl;
            cap.read(img);
            // image size is [2560 x 1440]
            Mat img_crop, img_resize;
            
            birdeye_transform(&img, cameras, &multi_frame_set, &vanishing_pts_set);
            cv::waitKey(20);
        }
        vanishing_pts final_vpts;
        cluster_vpt_set(&vanishing_pts_set, &final_vpts);
        bool result = calibration(frame_num, cameras_v, &multi_frame_set, &final_vpts);
        if (result == true) {
            for (int i = 0; i < frame_num; i ++) {
                std::vector<lanemarks> lines_orig;
                lines_orig.insert(lines_orig.end(), multi_frame_set[i].pre_filtered_lines->front_lanes.begin(), multi_frame_set[i].pre_filtered_lines->front_lanes.end());
                lines_orig.insert(lines_orig.end(), multi_frame_set[i].pre_filtered_lines->rear_lanes.begin(), multi_frame_set[i].pre_filtered_lines->rear_lanes.end());
                lines_orig.insert(lines_orig.end(), multi_frame_set[i].pre_filtered_lines->left_lanes.begin(), multi_frame_set[i].pre_filtered_lines->left_lanes.end());
                lines_orig.insert(lines_orig.end(), multi_frame_set[i].pre_filtered_lines->right_lanes.begin(), multi_frame_set[i].pre_filtered_lines->right_lanes.end());
                
                Mat img = Mat::zeros(Size(352, 704),CV_8UC1);
                for (uint32_t i = 0; i < lines_orig.size(); i++) {
                    cv::Vec4f line = lines_orig[i].rising_edge;
                    cv::circle(img, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255,255,255), 1);
                    cv::circle(img, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255,255,255), 1);
                    cv::line(img, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255,255,255), 1);
                    line = lines_orig[i].falling_edge;
                    cv::circle(img, cv::Point2f(line[0], line[1]), 1, cv::Scalar(255,255,255), 1);
                    cv::circle(img, cv::Point2f(line[2], line[3]), 1, cv::Scalar(255,255,255), 1);
                    cv::line(img, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), Scalar(255,255,255), 1);
                }
                cv::imshow("image", img);
                cv::waitKey(20);
            }
        }
        cout << "one cycle" << endl;
    }
    
    return 0;
  }
