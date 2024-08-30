#pragma once

#include <opencv2/core/affine.hpp>

class CameraModel
{
public:
    CameraModel(size_t ru, size_t rv, double pu, double pv, double fu, double fv, const cv::Affine3d &cam2rig = cv::Affine3d::Identity(), const cv::Mat &vignette = cv::Mat())
    : ru(ru), rv(rv), pu(pu), pv(pv), fu(fu), fv(fv), cam2rig(cam2rig), rig2cam(cam2rig.inv()), vignette(vignette), mask(vignette > 0) {}

    virtual cv::Point3d pixelToRay(const cv::Point2d &p) const = 0;
    virtual cv::Point2d rayToPixel(const cv::Point3d &r) const = 0;

    cv::Mat pixelToRay(const cv::Mat &p) const;
    cv::Mat rayToPixel(const cv::Mat &r) const;
    void pixelToRay(const cv::Mat &p, cv::Mat &r) const;
    void rayToPixel(const cv::Mat &r, cv::Mat &p) const;

    //TODO DEPRECATED
    cv::Mat rayToTheta(const cv::Mat &r) const;

    cv::Mat applyVignette(const cv::Mat &image) const;

    inline cv::Size size() const
    {
        return cv::Size(ru, rv);
    }
    inline size_t width() const
    {
        return ru;
    }
    inline size_t height() const
    {
        return rv;
    }

    static std::vector<std::shared_ptr<CameraModel>> load(const std::string &path);

    // resolution
    const size_t ru, rv;
    // principal point
    const double pu, pv;
    // focal-length
    const double fu, fv;

    // transformations
    const cv::Affine3d cam2rig;
    const cv::Affine3d rig2cam;

    // image vignette
    const cv::Mat vignette;
    const cv::Mat mask;

private:
    static std::vector<std::shared_ptr<CameraModel>> load_yaml(const std::string &path);
    static std::vector<std::shared_ptr<CameraModel>> load_json(const std::string &path);
};
