#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

namespace utils
{
    namespace geometry
    {
        /**
         * @brief Generates spherical rays based on equirectangular coordinates.
         *
         * This function calculates the X, Y, and Z components of spherical rays
         * given equirectangular coordinates, spherical angles (phi), and an
         * optional second angle (phi2).
         *
         * @param resolution Resultion of the sphere to generate rays
         * @param phi Angle in radiands of height
         * @param phi2 Angle in radiands of width or negative if ignored
         * @return cv::Mat A 2D vector containing X, Y, and Z components of spherical rays.
         */
        cv::Mat makeSphericalRays(const cv::Size &resolution, double phi, double phi2 = -1.0);

        cv::Mat transform(const cv::Affine3d &tf, const cv::Mat &points);

        void transform(const cv::Affine3d &tf, const cv::Mat &in, cv::Mat &out);

    } // namespace Geometry
} // namespace utils
