#include "camera_models/camera_model.hpp"
#include "camera_models/double_sphere.hpp"
#include "camera_models/extended_unified.hpp"

#include <exception>
#include <numeric>
#include <cmath>
#include <fstream>
#include <filesystem>

#include <opencv2/core/quaternion.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

template<typename float_type>
cv::Affine3<float_type> yamlToAffine(const YAML::Node &tf)
{
    float_type T_cn_cnm1[16];
    float_type *p_T = T_cn_cnm1;
    for (const YAML::Node &row : tf)
    {
        for (const YAML::Node &col : row)
        {
            *p_T = col.as<float_type>();
        }
    }
    return cv::Affine3<float_type>(T_cn_cnm1);
}

template<typename float_type>
cv::Affine3<float_type> jsonToAffine(const nlohmann::json &tf)
{
    float_type px = tf["px"].get<float_type>();
    float_type py = tf["py"].get<float_type>();
    float_type pz = tf["pz"].get<float_type>();

    typename cv::Affine3<float_type>::Vec3 t(px, py, pz);

    float_type qx = tf["qx"].get<float_type>();
    float_type qy = tf["qy"].get<float_type>();
    float_type qz = tf["qz"].get<float_type>();
    float_type qw = tf["qw"].get<float_type>();

    cv::Quat<float_type> quat(qw, qx, qy, qz);

    return cv::Affine3<float_type>(quat.toRotMat3x3(), t);
}

std::vector<std::shared_ptr<CameraModel>> CameraModel::load_yaml(const std::string &path)
{
    YAML::Node calib = YAML::LoadFile(path);
    std::vector<std::shared_ptr<CameraModel>> cams;
    cams.reserve(calib.size());

    cv::Affine3d cam2rig = cv::Affine3d::Identity();

    for (const auto &pair : calib)
    {
        const YAML::Node &cam = pair.second;
        const std::string &camera_model = cam["camera_model"].as<std::string>();
        const std::string &distortion_model = cam["distortion_model"].as<std::string>();
        const std::vector<size_t> &cam_overlaps = cam["cam_overlaps"].as<std::vector<size_t>>();
        const std::vector<size_t> &resolution = cam["resolution"].as<std::vector<size_t>>();
        const std::vector<double> &intrinsics = cam["intrinsics"].as<std::vector<double>>();
        const std::vector<double> &distortion_coeffs = cam["distortion_coeffs"].as<std::vector<double>>();

        if (cam["T_cn_cnm1"])
        {
            cam2rig = cam2rig.concatenate(yamlToAffine<double>(cam["T_cn_cnm1"]));
        }

        if (camera_model == "ds" && distortion_model == "none")
        {
            cams.push_back(std::make_shared<DoubleSphereModel>(resolution[0], resolution[1],
                                                                intrinsics[2], intrinsics[3],
                                                                intrinsics[4], intrinsics[5],
                                                                intrinsics[0], intrinsics[1],
                                                                cam2rig));
        }
        else
        {
            throw std::runtime_error("Camera model \"" + camera_model + "\" in combination with distortion model \"" + distortion_model + "\" is not supported.");
        }
    }

    return cams;
}

std::vector<std::shared_ptr<CameraModel>> CameraModel::load_json(const std::string &path)
{
    std::ifstream file(path);
    std::filesystem::path base_path = std::filesystem::path(path).parent_path();
    nlohmann::json calib = nlohmann::json::parse(file)["value0"];
    nlohmann::json tfs = calib["T_imu_cam"];
    nlohmann::json intr = calib["intrinsics"];
    nlohmann::json res = calib["resolution"];
    nlohmann::json ving = calib["vignette_file"];

    if (intr.size() != tfs.size() || intr.size() != res.size() || (!ving.is_null() && intr.size() != ving.size()))
    {
        throw std::runtime_error("Malformed calibration json, unable to read calibration.");
    }

    const size_t num_cams = intr.size();

    std::vector<std::shared_ptr<CameraModel>> cams;
    cams.reserve(num_cams);

    // Read tfs to imu and calculate their center of mass
    std::vector<cv::Affine3d> tfs_raw;
    tfs_raw.reserve(num_cams);
    cv::Affine3d::Vec3 center_of_mass(0, 0, 0);
    for (nlohmann::json &tf : tfs)
    {
        cv::Affine3d tf_raw = jsonToAffine<double>(tf);
        center_of_mass += tf_raw.translation();
        tfs_raw.push_back(tf_raw);
    }
    center_of_mass /= static_cast<double>(num_cams);
    for (cv::Affine3d &tf : tfs_raw)
    {
        tf.translation(tf.translation() - center_of_mass);
    }

    std::vector<cv::Affine3d>::iterator tfs_iter = tfs_raw.begin();
    nlohmann::json::iterator intr_iter = intr.begin();
    nlohmann::json::iterator res_iter = res.begin();
    nlohmann::json::iterator ving_iter = ving.begin();
    for (size_t i = 0; i < intr.size(); ++i)
    {
        nlohmann::json params = (*intr_iter)["intrinsics"];
        double fx = params["fx"].get<double>();
        double fy = params["fy"].get<double>();
        double cx = params["cx"].get<double>();
        double cy = params["cy"].get<double>();

        size_t cols = (*res_iter)[0].get<size_t>();
        size_t rows = (*res_iter)[1].get<size_t>();

        cv::Mat vignette;
        std::string vignette_name;
        if (!ving.is_null() && !ving_iter->is_null())
        {
            vignette_name = ving_iter->get<std::string>();
        }
        if (vignette_name.empty())
        {
            vignette_name = "vingette_" + std::to_string(i) + ".png";
        }
        cv::Mat vignette_raw = cv::imread(base_path / vignette_name, cv::IMREAD_GRAYSCALE);
        if (vignette_raw.empty())
        {
            throw std::runtime_error("Vignette is empty.");
        }
        //TODO get better vignette calibration to avoid preprocessing
        cv::Mat lables;
        cv::connectedComponents(vignette_raw, lables);
        vignette_raw.copyTo(vignette, lables == lables.at<int32_t>(lables.size() / 2));
        vignette.convertTo(vignette, CV_32FC1);
        vignette.forEach<float>([&](float &val, const int *pos) -> void {
            (void)pos;
            if (val != 0)
                val = 255.0 / val;
        });

        const std::string &camera_model = (*intr_iter)["camera_type"].get<std::string>();
        if (camera_model == "ds")
        {
            double xi = params["xi"].get<double>();
            double alpha = params["alpha"].get<double>();
            cams.push_back(std::make_shared<DoubleSphereModel>(cols, rows, cx, cy, fx, fy, xi, alpha, *tfs_iter, vignette));
        }
        else if (camera_model == "eucm")
        {
            double alpha = params["alpha"].get<double>();
            double beta = params["beta"].get<double>();
            cams.push_back(std::make_shared<ExtendedUnifiedModel>(cols, rows, cx, cy, fx, fy, alpha, beta, *tfs_iter, vignette));
        }
        else
        {
            throw std::runtime_error("Camera model \"" + camera_model + "\" is not supported.");
        }

        ++tfs_iter;
        ++intr_iter;
        ++res_iter;
        ++ving_iter;
    }

    return cams;
}

std::vector<std::shared_ptr<CameraModel>> CameraModel::load(const std::string &path)
{
    if (path.ends_with(".yaml") || path.ends_with(".yml"))
        return load_yaml(path);
    else if (path.ends_with(".json"))
        return load_json(path);
    else
        throw std::runtime_error("Only kalibr YAML files and basalt JSON files are supported.");
}

cv::Mat CameraModel::pixelToRay(const cv::Mat &p) const
{
    cv::Mat r;
    this->pixelToRay(p, r);
    return r;
}

cv::Mat CameraModel::rayToPixel(const cv::Mat &r) const
{
    cv::Mat p;
    this->rayToPixel(r, p);
    return p;
}

void CameraModel::pixelToRay(const cv::Mat &p, cv::Mat &r) const
{
    CV_Assert(p.type() == CV_32FC2);
    if (&p == &r)
    {
        r = this->pixelToRay(p);
    }
    else
    {
        r.create(p.rows, p.cols, CV_64FC3);
        r.forEach<cv::Point3d>([&](cv::Point3d& point, const int *pos) -> void {
            point = this->pixelToRay(p.at<cv::Point>(pos));
        });
    }
}

void CameraModel::rayToPixel(const cv::Mat &r, cv::Mat &p) const
{
    CV_Assert(r.type() == CV_64FC3);
    if (&r == &p)
    {
        p = this->rayToPixel(r);
    }
    else
    {
        p.create(r.rows, r.cols, CV_32FC2);
        p.forEach<cv::Point2f>([&](cv::Point2f& point, const int *pos) -> void {
            point = this->rayToPixel(r.at<cv::Point3d>(pos));
        });
    }
}

cv::Mat CameraModel::rayToTheta(const cv::Mat &r) const
{
    CV_Assert(r.type() == CV_64FC3);
    cv::Mat theta(r.rows, r.cols, CV_64F);
    theta.forEach<double>([&](double &val, const int *pos) -> void {
        const cv::Point3d &p = r.at<cv::Point3d>(pos);
        val = std::atan2(-p.z, std::sqrt(p.x*p.x + p.y*p.y) + std::numeric_limits<double>::epsilon());
    });
    return theta;
}

cv::Mat CameraModel::applyVignette(const cv::Mat &image) const
{
    const int color_depth = image.channels();

    cv::Mat corrected_image;
    image.convertTo(corrected_image, CV_32FC(color_depth));
    switch (color_depth) //TODO find a better way to do this
    {
    case 1:
        cv::multiply(corrected_image, this->vignette, corrected_image);
        break;
    case 2:
        corrected_image.forEach<cv::Vec2f>([&](cv::Vec2f &val, const int *pos) -> void {
            val *= this->vignette.at<float>(pos); });
        break;
    case 3:
        corrected_image.forEach<cv::Vec3f>([&](cv::Vec3f &val, const int *pos) -> void {
            val *= this->vignette.at<float>(pos); });
        break;
    case 4:
        corrected_image.forEach<cv::Vec4f>([&](cv::Vec4f &val, const int *pos) -> void {
            val *= this->vignette.at<float>(pos); });
        break;
    default:
        throw std::runtime_error("Only 1-4 channel images are supported for vignette correction.");
        break;
    }
    cv::add(corrected_image, 0.5, corrected_image);
    corrected_image.convertTo(corrected_image, CV_8UC(color_depth));
    return corrected_image;
}
