#pragma once

#include "camera_model.hpp"

class DoubleSphereModel : public CameraModel
{
public:
    DoubleSphereModel(size_t ru, size_t rv, double pu, double pv, double fu, double fv, double xi, double alpha, const cv::Affine3d &cam2rig = cv::Affine3d::Identity(), const cv::Mat &vignette = cv::Mat());

    cv::Point3d pixelToRay(const cv::Point2d &p) const;
    cv::Point2d rayToPixel(const cv::Point3d &r) const;

    // calibration parameters
    const double xi, alpha;
};
