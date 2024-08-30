#include <string>
#include <vector>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/imgproc.hpp>

#include "camera_models/camera_model.hpp"
#include "utils/geometry.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

class Insta360Stitcher
{
public:
    Insta360Stitcher(const std::string &camera_calibration_file, int output_height, double stitching_depth = 1.0, double roll_correction = 0.0)
    {
        cams_ = CameraModel::load(camera_calibration_file);
        tables_ = std::vector<cv::Mat>(cams_.size());
        res_ = cv::Size(output_height * 2, output_height);
        stitching_depth_ = stitching_depth;
        camera_roll_ = roll_correction;

        build_table();
    }

    void stitch(const std::vector<cv::Mat> &images, cv::Mat &output)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        const int color_depth = images[0].channels();

        cv::Mat pano = cv::Mat::zeros(res_.height, res_.width, CV_32FC(color_depth));
        cv::Mat count = cv::Mat::zeros(res_.height, res_.width, CV_32FC(color_depth));

        for (size_t i = 0; i < cams_.size(); ++i)
        {
            const cv::Mat &pixel = tables_[i];
            cv::Mat equi;
            cv::remap(images[i], equi, pixel, cv::noArray(), cv::INTER_NEAREST);

            cv::Mat mask;
            cv::remap(cams_[i]->mask, mask, pixel, cv::noArray(), cv::INTER_NEAREST);

            equi.convertTo(equi, CV_32FC(color_depth));
            cv::add(pano, equi, pano, mask);
            cv::add(count, 1, count, mask);
        }
        cv::divide(pano, count, pano);
        cv::add(pano, 0.5, pano); // round
        pano.convertTo(output, CV_8UC(color_depth));
    }

    pybind11::array_t<uint8_t, pybind11::array::c_style | pybind11::array::forcecast> stitch(pybind11::array_t<uint8_t, pybind11::array::c_style | pybind11::array::forcecast> input1, pybind11::array_t<uint8_t, pybind11::array::c_style | pybind11::array::forcecast> input2)
    {
        std::vector<cv::Mat> images;
        images.reserve(2);
        images.emplace_back(input1.shape(0), input1.shape(1), CV_8UC3, input1.mutable_data());
        images.emplace_back(input2.shape(0), input2.shape(1), CV_8UC3, input2.mutable_data());

        const int color_depth = images[0].channels();

        auto output = pybind11::array_t<uint8_t, pybind11::array::c_style | pybind11::array::forcecast>({res_.height, res_.width, color_depth});
        cv::Mat out(res_, images[0].type(), output.mutable_data());

        this->stitch(images, out);

        return output;
    }

    double camera_roll(double rad)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        camera_roll_ = rad;
        build_table();
        return camera_roll_;
    }
    double camera_roll() const
    {
        return camera_roll_;
    }

    double stitching_depth(double meter)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        stitching_depth_ = meter;
        build_table();
        return stitching_depth_;
    }
    double stitching_depth() const
    {
        return stitching_depth_;
    }
    cv::Size resolution(const cv::Size &res)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        res_ = res;
        build_table();
        return res_;
    }
    cv::Size resolution(int width, int height)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        res_ = cv::Size(width, height);
        build_table();
        return res_;
    }
    cv::Size resolution() const
    {
        return res_;
    }

    std::tuple<int, int> resolution_wrapper(const std::tuple<int, int> &res)
    {
        resolution(std::get<0>(res), std::get<1>(res));
        return std::make_tuple(res_.width, res_.height);
    }
    std::tuple<int, int> resolution_wrapper(int width, int height)
    {
        resolution(width, height);
        return std::make_tuple(res_.width, res_.height);
    }
    std::tuple<int, int> resolution_wrapper() const
    {
        return std::make_tuple(res_.width, res_.height);
    }

private:
    std::vector<std::shared_ptr<CameraModel>> cams_;
    std::vector<cv::Mat> tables_;

    std::mutex mtx_;
    cv::Size res_;
    double camera_roll_;
    double stitching_depth_;

    void build_table()
    {
        cv::Mat rays_unit = utils::geometry::makeSphericalRays(res_, M_PI_2, -1);
        cv::Mat rays_world = utils::geometry::transform(cv::Affine3d(cv::Vec3d(0, 0, camera_roll_)), rays_unit) * stitching_depth_;

        for (size_t i = 0; i < cams_.size(); ++i)
        {
            cv::Mat rays_cam = utils::geometry::transform(cams_[i]->rig2cam, rays_world);
            tables_[i] = cams_[i]->rayToPixel(rays_cam);
        }
    }
};


PYBIND11_MODULE(insta360_stitcher, m) {
    pybind11::class_<Insta360Stitcher>(m, "Insta360Stitcher")
        .def(pybind11::init<const std::string &, int, double, double>(),
            pybind11::arg("camera_calibration_file"), pybind11::arg("output_height"), pybind11::arg("stitching_depth") = 1.0, pybind11::arg("roll_correction") = 0.0)
        .def("stitch", static_cast<pybind11::array_t<uint8_t, pybind11::array::c_style | pybind11::array::forcecast> (Insta360Stitcher::*)(pybind11::array_t<uint8_t, pybind11::array::c_style | pybind11::array::forcecast>, pybind11::array_t<uint8_t, pybind11::array::c_style | pybind11::array::forcecast>)>(&Insta360Stitcher::stitch))
        .def("camera_roll", static_cast<double (Insta360Stitcher::*)(double)>(&Insta360Stitcher::camera_roll))
        .def("camera_roll", static_cast<double (Insta360Stitcher::*)() const>(&Insta360Stitcher::camera_roll))
        .def("stitching_depth", static_cast<double (Insta360Stitcher::*)(double)>(&Insta360Stitcher::stitching_depth))
        .def("stitching_depth", static_cast<double (Insta360Stitcher::*)() const>(&Insta360Stitcher::stitching_depth))
        .def("resolution", static_cast<std::tuple<int, int> (Insta360Stitcher::*)(const std::tuple<int, int>&)>(&Insta360Stitcher::resolution_wrapper))
        .def("resolution", static_cast<std::tuple<int, int> (Insta360Stitcher::*)(int, int)>(&Insta360Stitcher::resolution_wrapper))
        .def("resolution", static_cast<std::tuple<int, int> (Insta360Stitcher::*)() const>(&Insta360Stitcher::resolution_wrapper));
}
