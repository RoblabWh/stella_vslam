#ifndef STELLA_VSLAM_UTIL_KEYFRAME_CONVERTER_H
#define STELLA_VSLAM_UTIL_KEYFRAME_CONVERTER_H

#include "stella_vslam/data/keyframe.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace stella_vslam {
namespace util {

void convert_to_patch_match(const data::keyframe& keyfrm, cv::Matx33d& R_cw, cv::Vec3d& t_cw, std::vector<cv::Vec3d>& landmarks_w);

} // namespace util
} // namespace stella_vslam

#endif // STELLA_VSLAM_UTIL_KEYFRAME_CONVERTER_H
