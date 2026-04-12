#include "tracker_backend.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <limits>

namespace {

    std::string trim(const std::string& s) {
        const auto a = s.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) return "";
        const auto b = s.find_last_not_of(" \t\r\n");
        return s.substr(a, b - a + 1);
    }

    double toDouble(const std::map<std::string, std::string>& kv, const std::string& key, double def) {
        auto it = kv.find(key);
        if (it == kv.end()) return def;
        try { return std::stod(it->second); }
        catch (...) { return def; }
    }

    int toInt(const std::map<std::string, std::string>& kv, const std::string& key, int def) {
        auto it = kv.find(key);
        if (it == kv.end()) return def;
        try { return std::stoi(it->second); }
        catch (...) { return def; }
    }

    bool stereoUsable(const DetectionResult& L, const DetectionResult& R) {
        if (!L.valid || !R.valid) return false;
        const double du = L.center.x - R.center.x;
        return std::isfinite(du) && std::abs(du) >= 1.0;
    }

    double stereoSoftScore(const DetectionResult& L, const DetectionResult& R, int W, int H) {
        if (!stereoUsable(L, R)) return -1e12;

        const double du = L.center.x - R.center.x;
        const double dv = L.center.y - R.center.y;

        const double mL = std::min({ L.center.x, L.center.y, W - L.center.x, H - L.center.y });
        const double mR = std::min({ R.center.x, R.center.y, W - R.center.x, H - R.center.y });
        const double margin = std::max(10.0, 0.055 * std::min(W, H));
        const double penB = std::max(0.0, margin - mL) + std::max(0.0, margin - mR);

        const double dvTol = std::max(16.0, 0.055 * H);
        const double penV = std::max(0.0, std::abs(dv) - dvTol) * 8.0;

        const double rL = std::max(L.radius_px, 1.0);
        const double rR = std::max(R.radius_px, 1.0);
        const double penR = std::abs(rL - rR) / std::max(rL, rR);

        return 2.0 * std::min(mL, mR)
            + std::log1p(std::abs(du))
            + 22.0 * (1.0 - std::min(1.0, penR))
            - 3.0 * penB
            - penV;
    }

    void appendLine(std::ostringstream& oss, const std::string& line) {
        if (!oss.str().empty()) oss << " | ";
        oss << line;
    }

    cv::Point2d offsetPoint(const cv::Point2d& p, int x0, int y0) {
        return cv::Point2d(p.x + x0, p.y + y0);
    }

    cv::Matx33d makeK(double fx_px, double fy_px, double cx, double cy)
    {
        return cv::Matx33d(
            fx_px, 0.0, cx,
            0.0, fy_px, cy,
            0.0, 0.0, 1.0
        );
    }

    cv::Matx34d makeProjectionFromBlender(
        const cv::Matx33d& K,
        const cv::Matx33d& R_cw_bl,
        const cv::Vec3d& C_world)
    {
        // Blender camera coords -> OpenCV camera coords
        // Blender: +X right, +Y up, camera looks along -Z
        // OpenCV:  +X right, +Y down, +Z forward
        const cv::Matx33d S(
            1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0
        );

        // Blender gives camera-to-world rotation
        const cv::Matx33d R_wc_bl = R_cw_bl.t();
        const cv::Matx33d R_wc_cv = S * R_wc_bl;
        const cv::Vec3d t_cv = -(R_wc_cv * C_world);

        cv::Matx34d Rt(
            R_wc_cv(0, 0), R_wc_cv(0, 1), R_wc_cv(0, 2), t_cv(0),
            R_wc_cv(1, 0), R_wc_cv(1, 1), R_wc_cv(1, 2), t_cv(1),
            R_wc_cv(2, 0), R_wc_cv(2, 1), R_wc_cv(2, 2), t_cv(2)
        );

        return K * Rt;
    }

    cv::Vec3d triangulateWorldPoint(
        const cv::Point2d& pL,
        const cv::Point2d& pR,
        const cv::Matx34d& P1,
        const cv::Matx34d& P2)
    {
        cv::Mat pts1(2, 1, CV_64F);
        cv::Mat pts2(2, 1, CV_64F);

        pts1.at<double>(0, 0) = pL.x;
        pts1.at<double>(1, 0) = pL.y;
        pts2.at<double>(0, 0) = pR.x;
        pts2.at<double>(1, 0) = pR.y;

        cv::Mat X4;
        cv::triangulatePoints(P1, P2, pts1, pts2, X4);

        const double w = X4.at<double>(3, 0);
        if (!std::isfinite(w) || std::abs(w) < 1e-12) {
            return cv::Vec3d(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()
            );
        }

        return cv::Vec3d(
            X4.at<double>(0, 0) / w,
            X4.at<double>(1, 0) / w,
            X4.at<double>(2, 0) / w
        );
    }

    cv::Vec3d sanitizeWorld(const cv::Vec3d& Xw) {
        cv::Vec3d out = Xw;

        if (!std::isfinite(out[0]) || !std::isfinite(out[1]) || !std::isfinite(out[2])) {
            return cv::Vec3d(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()
            );
        }

        const double courtGroundZ = 0.35951;
        if (out[2] < courtGroundZ) {
            out[2] = courtGroundZ;
        }

        return out;
    }

} // namespace

std::map<std::string, std::string> parseKeyValueFile(const std::string& filename, std::string& err) {
    std::map<std::string, std::string> out;
    std::ifstream fin(filename);
    if (!fin) {
        err = "Could not open key/value file: " + filename;
        return out;
    }

    std::string line;
    while (std::getline(fin, line)) {
        line = trim(line);
        if (line.empty()) continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(line.substr(0, eq));
        const std::string val = trim(line.substr(eq + 1));
        out[key] = val;
    }
    return out;
}

bool loadConfig(const std::string& filename, Config& cfg, std::string& err) {
    const auto kv = parseKeyValueFile(filename, err);
    if (!err.empty()) return false;

    cfg.fx_px = toDouble(kv, "fx_px", cfg.f_px);
    cfg.fy_px = toDouble(kv, "fy_px", cfg.f_px);    
    cfg.pair1_baseline_m = toDouble(kv, "pair1_baseline_m", cfg.pair1_baseline_m);
    cfg.pair2_baseline_m = toDouble(kv, "pair2_baseline_m", cfg.pair2_baseline_m);
    cfg.net_v_frac = toDouble(kv, "net_v_frac", cfg.net_v_frac);
    cfg.radius_diff_px = toDouble(kv, "radius_diff_px", cfg.radius_diff_px);
    cfg.image_width = toInt(kv, "image_width", cfg.image_width);
    cfg.image_height = toInt(kv, "image_height", cfg.image_height);
    cfg.cx = toDouble(kv, "cx", cfg.cx);
    cfg.cy = toDouble(kv, "cy", cfg.cy);
    cfg.pair1_camBaseX = toDouble(kv, "pair1_camBaseX", cfg.pair1_camBaseX);
    cfg.pair1_camBaseY = toDouble(kv, "pair1_camBaseY", cfg.pair1_camBaseY);
    cfg.pair1_camBaseZ = toDouble(kv, "pair1_camBaseZ", cfg.pair1_camBaseZ);
    cfg.pair1_camPitchDeg = toDouble(kv, "pair1_camPitchDeg", cfg.pair1_camPitchDeg);
    cfg.pair2_camBaseX = toDouble(kv, "pair2_camBaseX", cfg.pair2_camBaseX);
    cfg.pair2_camBaseY = toDouble(kv, "pair2_camBaseY", cfg.pair2_camBaseY);
    cfg.pair2_camBaseZ = toDouble(kv, "pair2_camBaseZ", cfg.pair2_camBaseZ);
    cfg.pair2_camPitchDeg = toDouble(kv, "pair2_camPitchDeg", cfg.pair2_camPitchDeg);
    return true;
}

bool writeResultFile(const std::string& filename, const StereoOutput& out, std::string& err) {
    std::ofstream fout(filename);
    if (!fout) {
        err = "Could not open result file for write: " + filename;
        return false;
    }

    fout << std::setprecision(15);
    fout << "status=" << (out.ok ? "ok" : "error") << "\n";
    fout << "pair_index=" << out.pair_index << "\n";
    fout << "court_side=" << out.court_side << "\n";
    fout << "left_u=" << out.left.x << "\n";
    fout << "left_v=" << out.left.y << "\n";
    fout << "right_u=" << out.right.x << "\n";
    fout << "right_v=" << out.right.y << "\n";
    fout << "disparity_u=" << out.disparity_u << "\n";
    fout << "disparity_v=" << out.disparity_v << "\n";
    fout << "X=" << out.X << "\n";
    fout << "Y=" << out.Y << "\n";
    fout << "Z=" << out.Z << "\n";
    fout << "message=" << out.message << "\n";
    return true;
}

DetectionResult detectTennisBallYCbCr(const cv::Mat& bgr, const std::string& tag) {
    DetectionResult out;
    if (bgr.empty()) {
        out.debug = tag + ": empty image";
        return out;
    }

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat ballMask;
    cv::inRange(hsv, cv::Scalar(5, 120, 120), cv::Scalar(22, 255, 255), ballMask);

    cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(ballMask, ballMask, cv::MORPH_OPEN, se);

    cv::imwrite(tag + "_mask.png", ballMask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(ballMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        out.debug = tag + ": no contours after orange HSV mask";
        return out;
    }

    const double expectedR = 11.0;
    const double areaExp = CV_PI * expectedR * expectedR;
    double bestScore = -1e18;
    size_t bestIdx = 0;

    for (size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        const cv::Rect box = cv::boundingRect(contours[i]);

        cv::Point2f cTmp;
        float rTmp = 0.0f;
        cv::minEnclosingCircle(contours[i], cTmp, rTmp);

        const double perimeter = std::max(cv::arcLength(contours[i], true), 1.0);
        const double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        const double aspect = static_cast<double>(box.width) / std::max(box.height, 1);

        std::cout << tag
            << " contour " << i
            << " area=" << area
            << " box=(" << box.x << "," << box.y << "," << box.width << "," << box.height << ")"
            << " center=(" << cTmp.x << "," << cTmp.y << ")"
            << " r=" << rTmp
            << " circ=" << circularity
            << " aspect=" << aspect
            << std::endl;

        if (area < 20.0) {
            std::cout << "  rejected: area too small\n";
            continue;
        }
        if (area > 1000.0) {
            std::cout << "  rejected: area too large\n";
            continue;
        }

        if (box.x <= 1 || box.y <= 1 ||
            (box.x + box.width) >= (bgr.cols - 1) ||
            (box.y + box.height) >= (bgr.rows - 1)) {
            std::cout << "  rejected: touches border\n";
            continue;
        }

        if (cTmp.y < 0.05f * bgr.rows) {
            std::cout << "  rejected: too high in image\n";
            continue;
        }

        if (rTmp < 3.0f || rTmp > 20.0f) {
            std::cout << "  rejected: radius out of range\n";
            continue;
        }

        if (circularity < 0.05) {
            std::cout << "  rejected: circularity too low\n";
            continue;
        }

        const cv::Point2d ctr(box.x + 0.5 * box.width, box.y + 0.5 * box.height);
        const cv::Point2d ref(0.5 * bgr.cols, 0.78 * bgr.rows);
        const double d2 = (ctr.x - ref.x) * (ctr.x - ref.x) + (ctr.y - ref.y) * (ctr.y - ref.y);
        const double prox = 1.0 / (1.0 + d2);
        const double areaScore = 1.0 - std::min(std::abs(std::log(std::max(area / areaExp, 0.1))), 2.0) / 2.0;

        const double score = 120.0 * circularity
            + 80.0 * areaScore
            - 25.0 * std::abs(1.0 - aspect)
            + 5000.0 * prox;

        std::cout << "  accepted for scoring: score=" << score
            << " areaScore=" << areaScore
            << " prox=" << prox
            << std::endl;

        if (score > bestScore) {
            bestScore = score;
            bestIdx = i;
        }
    }

    if (bestScore <= -1e17) {
        out.debug = tag + ": no valid contour survived scoring";
        return out;
    }

    cv::Point2f c;
    float r = 0.0f;
    cv::minEnclosingCircle(contours[bestIdx], c, r);
    out.valid = std::isfinite(c.x) && std::isfinite(c.y) && r > 0.0f;
    out.center = cv::Point2d(c.x, c.y);
    out.radius_px = r;

    std::ostringstream oss;
    oss << tag << ": orange HSV mask center=(" << c.x << ", " << c.y << ") r=" << r
        << " contours=" << contours.size();
    out.debug = oss.str();
    return out;
}

DetectionResult refineWithHoughCircles(const cv::Mat& bgr, const DetectionResult& coarse, const std::string& tag) {
    if (!coarse.valid) return coarse;

    DetectionResult out = coarse;

    const int roiHalf = std::max(24, static_cast<int>(std::round(2.8 * std::max(coarse.radius_px, 4.0))));
    const int x1 = std::max(0, static_cast<int>(std::floor(coarse.center.x - roiHalf)));
    const int y1 = std::max(0, static_cast<int>(std::floor(coarse.center.y - roiHalf)));
    const int x2 = std::min(bgr.cols - 1, static_cast<int>(std::ceil(coarse.center.x + roiHalf)));
    const int y2 = std::min(bgr.rows - 1, static_cast<int>(std::ceil(coarse.center.y + roiHalf)));
    if (x2 <= x1 + 8 || y2 <= y1 + 8) {
        out.debug += " | " + tag + ": ROI too small; using coarse";
        return out;
    }

    cv::Rect roi(x1, y1, x2 - x1 + 1, y2 - y1 + 1);
    cv::Mat crop = bgr(roi).clone();
    cv::Mat gray;
    cv::cvtColor(crop, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.8);

    const int minR = std::max(2, static_cast<int>(std::floor(0.65 * std::max(coarse.radius_px, 4.0))));
    const int maxR = std::max(minR + 2, static_cast<int>(std::ceil(1.40 * std::max(coarse.radius_px, 4.0))));

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1.0, std::max(8.0, coarse.radius_px), 60.0, 10.0, minR, maxR);

    if (circles.empty()) {
        out.debug += " | " + tag + ": Hough none; using coarse";
        return out;
    }

    double bestD2 = 1e18;
    cv::Vec3f best = circles.front();
    for (const auto& c : circles) {
        const double dx = c[0] - (coarse.center.x - x1);
        const double dy = c[1] - (coarse.center.y - y1);
        const double d2 = dx * dx + dy * dy;
        if (d2 < bestD2) {
            bestD2 = d2;
            best = c;
        }
    }

    out.valid = true;
    out.center = offsetPoint(cv::Point2d(best[0], best[1]), x1, y1);
    out.radius_px = best[2];

    std::ostringstream oss;
    oss << out.debug << " | " << tag << ": Hough refined center=("
        << out.center.x << ", " << out.center.y << ") r=" << out.radius_px;
    out.debug = oss.str();
    return out;
}

StereoOutput processFourViews(const cv::Mat& p1L,
    const cv::Mat& p1R,
    const cv::Mat& p2L,
    const cv::Mat& p2R,
    const Config& cfg) {
    StereoOutput out;
    std::ostringstream dbg;

    auto c1L = detectTennisBallYCbCr(p1L, "pair1_left");
    auto c1R = detectTennisBallYCbCr(p1R, "pair1_right");
    auto c2L = detectTennisBallYCbCr(p2L, "pair2_left");
    auto c2R = detectTennisBallYCbCr(p2R, "pair2_right");

    appendLine(dbg, c1L.debug);
    appendLine(dbg, c1R.debug);
    appendLine(dbg, c2L.debug);
    appendLine(dbg, c2R.debug);

    const bool ok1 = stereoUsable(c1L, c1R);
    const bool ok2 = stereoUsable(c2L, c2R);

    if (!ok1 && !ok2) {
        out.message = dbg.str() + " | Neither stereo pair usable.";
        return out;
    }

    int pairIdx = 0;
    std::string courtSide;

    if (ok1 && !ok2) {
        pairIdx = 1;
        courtSide = "Pair 1 side";
        appendLine(dbg, "Only pair 1 usable.");
    }
    else if (ok2 && !ok1) {
        pairIdx = 2;
        courtSide = "Pair 2 side";
        appendLine(dbg, "Only pair 2 usable.");
    }
    else {
        const double s1 = stereoSoftScore(c1L, c1R, cfg.image_width, cfg.image_height);
        const double s2 = stereoSoftScore(c2L, c2R, cfg.image_width, cfg.image_height);
        pairIdx = (s1 >= s2) ? 1 : 2;
        courtSide = (pairIdx == 1) ? "Pair 1 side" : "Pair 2 side";
        appendLine(dbg, "Pair chosen by soft stereo score only.");
    }

    const DetectionResult& L = (pairIdx == 1) ? c1L : c2L;
    const DetectionResult& R = (pairIdx == 1) ? c1R : c2R;

    const double du = L.center.x - R.center.x;
    const double dv = L.center.y - R.center.y;
    if (!std::isfinite(du) || std::abs(du) < 1.0) {
        out.message = dbg.str() + " | Disparity too small for stable depth.";
        return out;
    }

    const cv::Matx33d K = makeK(cfg.fx_px, cfg.fy_px, cfg.cx, cfg.cy);

    // Actual Blender camera centers
    const cv::Vec3d C1(-1.5, -15.0, 12.0);
    const cv::Vec3d C2(1.5, -15.0, 12.0);
    const cv::Vec3d C3(-1.5, -3.0, 12.0);
    const cv::Vec3d C4(1.5, -3.0, 12.0);

    // Actual normalized Blender camera-to-world rotations
    const cv::Matx33d R12_cw_bl(
        1.0, 0.0, 0.0,
        0.0, 0.8660254037844386, -0.5,
        0.0, 0.5, 0.8660254037844386
    );

    const cv::Matx33d R34_cw_bl(
        1.0, 0.0, 0.0,
        0.0, 0.7880107536067220, -0.6156614753256583,
        0.0, 0.6156614753256583, 0.7880107536067220
    );

    cv::Matx34d Pleft, Pright;
    cv::Vec3d CamL, CamR;

    if (pairIdx == 1) {
        Pleft = makeProjectionFromBlender(K, R12_cw_bl, C1);
        Pright = makeProjectionFromBlender(K, R12_cw_bl, C2);
        CamL = C1;
        CamR = C2;
    }
    else {
        Pleft = makeProjectionFromBlender(K, R34_cw_bl, C3);
        Pright = makeProjectionFromBlender(K, R34_cw_bl, C4);
        CamL = C3;
        CamR = C4;
    }

    cv::Vec3d Xw = triangulateWorldPoint(L.center, R.center, Pleft, Pright);
    Xw = sanitizeWorld(Xw);

    if (!std::isfinite(Xw[0]) || !std::isfinite(Xw[1]) || !std::isfinite(Xw[2])) {
        out.message = dbg.str() + " | Triangulation failed.";
        return out;
    }

    out.ok = true;
    out.pair_index = pairIdx;
    out.court_side = courtSide;
    out.left = L.center;
    out.right = R.center;
    out.disparity_u = du;
    out.disparity_v = dv;
    out.X = Xw[0];
    out.Y = Xw[1];
    out.Z = Xw[2];

    std::ostringstream msg;
    msg << dbg.str()
        << " | pair=" << pairIdx
        << " | CamL=(" << CamL[0] << "," << CamL[1] << "," << CamL[2] << ")"
        << " | CamR=(" << CamR[0] << "," << CamR[1] << "," << CamR[2] << ")"
        << " | worldXYZ=(" << out.X << ", " << out.Y << ", " << out.Z << ")";
    out.message = msg.str();

    return out;
}