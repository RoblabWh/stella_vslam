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

std::shared_ptr<dense_point> dense_point::from_stmt(sqlite3_stmt* stmt,
                                              std::unordered_map<unsigned int, std::shared_ptr<stella_vslam::data::keyframe>>& keyframes,
                                              unsigned int next_dense_point_id,
                                              unsigned int next_keyframe_id) {
    const char* p;
    int column_id = 0;
    auto id = sqlite3_column_int64(stmt, column_id);
    column_id++;
    Vec3_t pos_w;
    p = reinterpret_cast<const char*>(sqlite3_column_blob(stmt, column_id));
    std::memcpy(pos_w.data(), p, sqlite3_column_bytes(stmt, column_id));
    column_id++;
    Color_t color;
    p = reinterpret_cast<const char*>(sqlite3_column_blob(stmt, column_id));
    std::memcpy(color.data(), p, sqlite3_column_bytes(stmt, column_id));
    column_id++;
    auto ref_keyfrm_id = sqlite3_column_int64(stmt, column_id);
    column_id++;

    auto ref_keyfrm = ref_keyfrm_id != (uint32_t) -1 ? keyframes.at(ref_keyfrm_id + next_keyframe_id) : std::shared_ptr<keyframe>(nullptr);

    auto lm = std::make_shared<dense_point>(id + next_dense_point_id, pos_w, color, ref_keyfrm);
    return lm;
}

bool dense_point::bind_to_stmt(sqlite3* db, sqlite3_stmt* stmt) const {
    int ret = SQLITE_ERROR;
    int column_id = 1;
    ret = sqlite3_bind_int64(stmt, column_id++, id_);
    if (ret == SQLITE_OK) {
        const Vec3_t pos_w = get_pos_in_world();
        ret = sqlite3_bind_blob(stmt, column_id++, pos_w.data(), pos_w.rows() * pos_w.cols() * sizeof(decltype(pos_w)::Scalar), SQLITE_TRANSIENT);
    }
    if (ret == SQLITE_OK) {
        const Color_t color = get_color_in_bgr();
        ret = sqlite3_bind_blob(stmt, column_id++, color.data(), color.rows() * color.cols() * sizeof(decltype(color)::Scalar), SQLITE_TRANSIENT);
    }
    if (ret == SQLITE_OK) {
        ret = sqlite3_bind_int64(stmt, column_id++, ref_keyfrm_.expired() ? -1 : ref_keyfrm_.lock()->id_);
    }
    if (ret != SQLITE_OK) {
        spdlog::error("SQLite error (bind): {}", sqlite3_errmsg(db));
    }
    return ret == SQLITE_OK;
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
            {"ref_keyfrm", ref_keyfrm_.expired() ? -1 : ref_keyfrm_.lock()->id_}};
}

} // namespace data
} // namespace stella_vslam
