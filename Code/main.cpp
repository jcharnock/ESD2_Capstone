#include "tracker_backend.hpp"

#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc == 3 && std::string(argv[1]) == "--dummy") {
        const std::string resPath = argv[2];

        StereoOutput out;
        out.ok = true;
        out.pair_index = 1;
        out.court_side = "Pair 1 side";
        out.left = cv::Point2d(300.0, 200.0);
        out.right = cv::Point2d(260.0, 200.0);
        out.disparity_u = 40.0;
        out.disparity_v = 0.0;
        out.X = 0.10;
        out.Y = -7.50;
        out.Z = 1.50;
        out.message = "Dummy backend test successful.";

        std::string writeErr;
        if (!writeResultFile(resPath, out, writeErr)) {
            std::cerr << writeErr << "\n";
            return 5;
        }
        std::cout << out.message << "\n";
        return 0;
    }

    if (argc != 7) {
        std::cerr << "Usage: triangulation <p1L> <p1R> <p2L> <p2R> <config.txt> <result.txt>\n";
        std::cerr << "   or: triangulation --dummy <result.txt>\n";
        return 2;
    }

    const std::string p1LPath = argv[1];
    const std::string p1RPath = argv[2];
    const std::string p2LPath = argv[3];
    const std::string p2RPath = argv[4];
    const std::string cfgPath = argv[5];
    const std::string resPath = argv[6];

    Config cfg;
    std::string err;
    if (!loadConfig(cfgPath, cfg, err)) {
        StereoOutput out;
        out.ok = false;
        out.message = err;
        std::string writeErr;
        writeResultFile(resPath, out, writeErr);
        std::cerr << err << "\n";
        return 3;
    }

    cv::Mat p1L = cv::imread(p1LPath, cv::IMREAD_COLOR);
    cv::Mat p1R = cv::imread(p1RPath, cv::IMREAD_COLOR);
    cv::Mat p2L = cv::imread(p2LPath, cv::IMREAD_COLOR);
    cv::Mat p2R = cv::imread(p2RPath, cv::IMREAD_COLOR);

    if (p1L.empty() || p1R.empty() || p2L.empty() || p2R.empty()) {
        StereoOutput out;
        out.ok = false;
        out.message = "Failed to read one or more input images.";
        std::string writeErr;
        writeResultFile(resPath, out, writeErr);
        std::cerr << out.message << "\n";
        return 4;
    }

    StereoOutput out = processFourViews(p1L, p1R, p2L, p2R, cfg);

    std::string writeErr;
    if (!writeResultFile(resPath, out, writeErr)) {
        std::cerr << writeErr << "\n";
        return 5;
    }

    std::cout << out.message << "\n";
    return out.ok ? 0 : 1;
}
