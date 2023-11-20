#include "stella_vslam/data/keyframe.h"
#include "stella_vslam/data/map_database.h"
#include "stella_vslam/io/keyframe_io_jpg.h"
#include "stella_vslam/util/image_converter.h"

#include <spdlog/spdlog.h>

#include <opencv2/imgcodecs.hpp>

namespace stella_vslam {
namespace io {

bool keyframe_io_jpg::save(const std::string& path,
                           const data::map_database* const map_db) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    assert(map_db);

    auto keyfrms = map_db->get_all_keyframes();

    for (auto keyfrm : keyfrms)
    {
        if (!keyfrm->img_.empty())
        {
            cv::imwrite(path + std::string("image") + std::to_string(keyfrm->id_) + std::string(".jpg"), keyfrm->img_);
        }
        if (!keyfrm->depth_.empty())
        {
            cv::Mat depth;
            cv::Mat mask;
            util::resize(keyfrm->mask_, mask, keyfrm->depth_.rows, keyfrm->depth_.cols);
            cv::normalize(keyfrm->depth_, depth, 255, 0, cv::NORM_MINMAX, CV_8UC1, mask);
            cv::imwrite(path + std::string("depth") + std::to_string(keyfrm->id_) + std::string(".jpg"), depth);
        }
        if (!keyfrm->mask_.empty())
        {
            cv::imwrite(path + std::string("mask") + std::to_string(keyfrm->id_) + std::string(".jpg"), keyfrm->mask_);
        }
    }

    return true;
}

} // namespace io
} // namespace stella_vslam
