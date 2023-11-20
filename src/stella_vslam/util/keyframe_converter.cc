#include "stella_vslam/util/keyframe_converter.h"

#include "stella_vslam/data/landmark.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

namespace stella_vslam {
namespace util {

void convert_to_patch_match(const data::keyframe& keyfrm, cv::Matx33d& R_cw, cv::Vec3d& t_cw, std::vector<cv::Vec3d>& landmarks_w) {
	const Mat44_t pose_cw = keyfrm.get_pose_cw();
	const std::set<std::shared_ptr<data::landmark>> landmarks = keyfrm.get_valid_landmarks();

	cv::eigen2cv((Mat33_t) pose_cw.block<3, 3>(0, 0), R_cw);
	cv::eigen2cv((Vec3_t) pose_cw.block<3, 1>(0, 3), t_cw);

	landmarks_w.reserve(landmarks.size());
	for (const std::shared_ptr<data::landmark>& i : landmarks) {
		cv::Vec3d lm;
		cv::eigen2cv(i->get_pos_in_world(), lm);
		landmarks_w.push_back(lm);
	}
}

} // namespace util
} // namespace stella_vslam
