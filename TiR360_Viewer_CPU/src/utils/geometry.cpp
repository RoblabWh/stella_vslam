#include "utils/geometry.hpp"

#include <iostream>
#include <opencv2/imgproc.hpp>

namespace utils
{
    namespace geometry
    {
        cv::Mat makeSphericalRays(const cv::Size &resolution, double phi, double phi2)
        {
            // Initialize the vector to store X, Y, and Z components of rays
            cv::Mat rays(resolution.height, resolution.width, CV_64FC3);

            const double med = (phi2 - phi) / 2.0;
            const double med2 = (phi2 + phi) / 2.0;

            // Calculate adjustments for coordinates
            const double width_2 = resolution.width / 2.0;
            const double height_2 = (resolution.height - 1) / 2.0;

            rays.forEach<cv::Point3d>([&](cv::Point3d &point, const int *pos) -> void
                                    {
                                        const int u = pos[1];
                                        const int v = pos[0];
                                        const double x = (u - width_2) / width_2 * CV_PI + (CV_PI / 2.0);
                                        double y;
                                        if (phi2 > 0.0)
                                        {
                                            y = (v - height_2) / height_2 * med2 + med;
                                        }
                                        else
                                        {
                                            y = (v - height_2) / height_2 * phi;
                                        }

                                        // Calculate X, Y, and Z components of spherical rays
                                        point.x = -std::cos(y) * std::cos(x); // x
                                        point.y = std::sin(y);                // y
                                        point.z = std::cos(y) * std::sin(x);  // z
                                    });

            return rays;
        }

        cv::Mat transform(const cv::Affine3d &tf, const cv::Mat &points)
        {
            cv::Mat out(points.rows, points.cols, CV_64FC3);
            transform(tf, points, out);
            return out;
        }

        void transform(const cv::Affine3d &tf, const cv::Mat &in, cv::Mat &out)
        {
            CV_Assert(in.type() == CV_64FC3);
            if (&in == &out)
            {
                out = transform(tf, in);
            }
            else
            {
                out.create(in.size(), CV_64FC3);
                out.forEach<cv::Point3d>([&](cv::Point3d& point, const int *pos) -> void {
                    point = tf * in.at<cv::Point3d>(pos);
            });
            }
        }

    } // namespace geometry
} // namespace utils
