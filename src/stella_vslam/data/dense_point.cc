#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/dense_point.h"
#include "stella_vslam/data/map_database.h"

#include <nlohmann/json.hpp>

#include <spdlog/spdlog.h>

namespace stella_vslam {
namespace data {

dense_point::dense_point(unsigned int id, const Vec3_t& pos_w, const Color_t& color, const std::shared_ptr<keyframe>& ref_keyfrm)
    : id_(id), pos_w_(pos_w), color_(color), ref_keyfrm_(ref_keyfrm) {}

dense_point::~dense_point() {
    SPDLOG_TRACE("dense_point::~dense_point: {}", id_);
}

void dense_point::set_pos_in_world(const Vec3_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_);
    SPDLOG_TRACE("dense_point::set_pos_in_world {}", id_);
    pos_w_ = pos_w;
}

Vec3_t dense_point::get_pos_in_world() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return pos_w_;
}

void dense_point::set_color_in_rgb(const Color_t& color) {
    std::lock_guard<std::mutex> lock(mtx_);
    SPDLOG_TRACE("dense_point::set_color_in_rgb {}", id_);
    color_ = color.reverse();
}

Color_t dense_point::get_color_in_rgb() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return color_.reverse();
}

void dense_point::set_color_in_bgr(const Color_t& color) {
    std::lock_guard<std::mutex> lock(mtx_);
    SPDLOG_TRACE("dense_point::set_color_in_bgr {}", id_);
    color_ = color;
}

Color_t dense_point::get_color_in_bgr() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return color_;
}

std::shared_ptr<keyframe> dense_point::get_ref_keyframe() const {
    return ref_keyfrm_.lock();
}

nlohmann::json dense_point::to_json() const {
    return {{"pos_w", {pos_w_(0), pos_w_(1), pos_w_(2)}},
            {"color", {color_(2), color_(1), color_(0)}},
            {"ref_keyfrm", ref_keyfrm_.lock()->id_}};
}

} // namespace data
} // namespace stella_vslam
