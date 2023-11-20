#ifndef STELLA_VSLAM_DATA_DENSE_POINT_H
#define STELLA_VSLAM_DATA_DENSE_POINT_H

#include "stella_vslam/type.h"

#include <map>
#include <mutex>
#include <atomic>
#include <memory>

#include <opencv2/core/mat.hpp>
#include <nlohmann/json_fwd.hpp>
#include <sqlite3.h>

namespace stella_vslam {

using Color_t = Eigen::Matrix<uint8_t, 3, 1>;

namespace data {

class keyframe;

class map_database;

class dense_point : public std::enable_shared_from_this<dense_point> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    dense_point(unsigned int id, const Vec3_t& pos_w, const Color_t& color, const std::shared_ptr<keyframe>& ref_keyfrm);

    virtual ~dense_point();

    //! set world coordinates of this dense point
    void set_pos_in_world(const Vec3_t& pos_w);
    //! get world coordinates of this dense point
    Vec3_t get_pos_in_world() const;

    //! set color of this dense point in rgb
    void set_color_in_rgb(const Color_t& color);
    //! get color of this dense point in rgb
    Color_t get_color_in_rgb() const;

    //! set color of this dense point in bgr
    void set_color_in_bgr(const Color_t& color);
    //! get color of this dense point in bgr
    Color_t get_color_in_bgr() const;

    //! get reference keyframe, a keyframe at the creation of a given 3D point
    std::shared_ptr<keyframe> get_ref_keyframe() const;

    //! encode dense point information as JSON
    nlohmann::json to_json() const;

public:
    unsigned int id_;

private:
    //! world coordinates of this dense point
    Vec3_t pos_w_;

    //! color of this dense point
    Color_t color_;

    //! reference keyframe
    std::weak_ptr<keyframe> ref_keyfrm_;

    mutable std::mutex mtx_;
};

} // namespace data
} // namespace stella_vslam

#endif // STELLA_VSLAM_DATA_DENSE_POINT_H
