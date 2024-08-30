#include "camera_models/double_sphere.hpp"

#include <numeric>

DoubleSphereModel::DoubleSphereModel(size_t ru, size_t rv, double pu, double pv, double fu, double fv, double xi, double alpha, const cv::Affine3d &cam2rig, const cv::Mat &vignette)
: CameraModel(ru, rv, pu, pv, fu, fv, cam2rig, vignette), xi(xi), alpha(alpha) {}

cv::Point3d DoubleSphereModel::pixelToRay(const cv::Point2d &p) const
{
    const double mx = (p.x - this->pu) / this->fu;
    const double my = (p.y - this->pv) / this->fv;

    const double r2 = mx * mx + my * my;

    // check if unprojected point is valid
    if (this->alpha > 0.5 && (r2 >= 1.0 / (2 * this->alpha - 1)))
        return cv::Point3d(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());

    const double xi2_2 = this->alpha * this->alpha;
    const double sqrt2 = std::sqrt(1 - (2 * this->alpha - 1) * r2);
    const double norm2 = this->alpha * sqrt2 + 1 - this->alpha;

    const double mz = (1 - xi2_2 * r2) / norm2;
    const double mz2 = mz * mz;

    const double xi1_2 = this->xi * this->xi;
    const double sqrt1 = std::sqrt(mz2 + (1 - xi1_2) * r2);
    const double norm1 = mz2 + r2;
    const double k = (mz * this->xi + sqrt1) / norm1;

    const double x = k * mx;
    const double y = k * my;
    const double z = k * mz - this->xi;

    return cv::Point3d(x, y, z);
}

cv::Point2d DoubleSphereModel::rayToPixel(const cv::Point3d &r) const
{
    const double x = r.x;
    const double y = r.y;
    const double z = r.z;

    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;

    const double r2 = xx + yy;

    const double d1 = std::sqrt(r2 + zz);

    const double w1 = this->alpha <= 0.5 ? this->alpha / (1 - this->alpha) : (1 - this->alpha) / this->alpha;
    const double w2 = (w1 + this->xi) / std::sqrt(2 * w1 * this->xi + this->xi * this->xi + 1);

    // Check if point will lead to a valid projection
    if (z <= -(w2 * d1))
        return cv::Point2d(-1e5, -1e5);

    const double k = this->xi * d1 + z;
    const double kk = k * k;

    const double d2 = std::sqrt(r2 + kk);

    const double norm = this->alpha * d2 + (1 - this->alpha) * k;

    const double u = this->fu * (x / norm) + this->pu;
    const double v = this->fv * (y / norm) + this->pv;

    return cv::Point2d(u, v);
}
