#pragma once

#include <opencv2/opencv.hpp>
#include <map>
#include <string>

struct Config {
    double f_px = 430.0;
    double pair1_baseline_m = 3.0;
    double pair2_baseline_m = 3.0;
    double net_v_frac = 0.42;
    double radius_diff_px = 2.0;
    int image_width = 1280;
    int image_height = 720;
    double cx = 640.0;
    double cy = 360.0;

    double pair1_camBaseX = 0.0;
    double pair1_camBaseY = -13.0;
    double pair1_camBaseZ = 12.0;
    double pair1_camPitchDeg = 30.0;

    double pair2_camBaseX = 0.0;
    double pair2_camBaseY = -2.0;
    double pair2_camBaseZ = 12.0;
    double pair2_camPitchDeg = 38.0;
};

struct DetectionResult {
    bool valid = false;
    cv::Point2d center{ 0.0, 0.0 };
    double radius_px = 0.0;
    std::string debug;
};

struct StereoOutput {
    bool ok = false;
    int pair_index = 0;
    std::string court_side = "--";
    cv::Point2d left{ 0.0, 0.0 };
    cv::Point2d right{ 0.0, 0.0 };
    double disparity_u = 0.0;
    double disparity_v = 0.0;
    double X = 0.0;
    double Y = 0.0;
    double Z = 0.0;
    std::string message;
};

std::map<std::string, std::string> parseKeyValueFile(const std::string& filename, std::string& err);
bool loadConfig(const std::string& filename, Config& cfg, std::string& err);
bool writeResultFile(const std::string& filename, const StereoOutput& out, std::string& err);
DetectionResult detectTennisBallYCbCr(const cv::Mat& bgr, const std::string& tag);
DetectionResult refineWithHoughCircles(const cv::Mat& bgr, const DetectionResult& coarse, const std::string& tag);
StereoOutput processFourViews(const cv::Mat& p1L, const cv::Mat& p1R, const cv::Mat& p2L, const cv::Mat& p2R, const Config& cfg);
