#include "camera_models/extended_unified.hpp"

#include <numeric>

ExtendedUnifiedModel::ExtendedUnifiedModel(size_t ru, size_t rv, double pu, double pv, double fu, double fv, double alpha, double beta, const cv::Affine3d &cam2rig, const cv::Mat &vignette)
: CameraModel(ru, rv, pu, pv, fu, fv, cam2rig, vignette), alpha(alpha), beta(beta) {}

cv::Point3d ExtendedUnifiedModel::pixelToRay(const cv::Point2d &p) const
{
    const double mx = (p.x - this->pu) / this->fu;
    const double my = (p.y - this->pv) / this->fv;

    const double r2 = mx * mx + my * my;
    const double gamma = 1 - this->alpha;

    // check if unprojected point is valid
    if (this->alpha > 0.5 && (r2 >= 1.0 / ((this->alpha - gamma) * this->beta)))
        return cv::Point3d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());

    const double tmp1 = 1.0 - this->alpha * this->alpha * this->beta * r2;
    const double tmp_sqrt = std::sqrt(1.0 - (this->alpha - gamma) * this->beta * r2);
    const double tmp2 = this->alpha * tmp_sqrt + gamma;

    const double k = tmp1 / tmp2;

    const double norm = std::sqrt(r2 + k * k);

    const double x = mx / norm;
    const double y = my / norm;
    const double z = k / norm;

    return cv::Point3d(x, y, z);
}

cv::Point2d ExtendedUnifiedModel::rayToPixel(const cv::Point3d &r) const
{
    const double x = r.x;
    const double y = r.y;
    const double z = r.z;

    const double r2 = x * x + y * y;

    const double rho = std::sqrt(this->beta * r2 + z * z);

    const double w = this->alpha <= 0.5 ? this->alpha / (1 - this->alpha) : (1 - this->alpha) / this->alpha;
    // Check if point will lead to a valid projection
    if (z <= -(w * rho))
        return cv::Point2d(-1e5, -1e5);

    const double norm = this->alpha * rho + (1 - this->alpha) * z;

    const double u = this->fu * (x / norm) + this->pu;
    const double v = this->fv * (y / norm) + this->pv;

    return cv::Point2d(u, v);
}
