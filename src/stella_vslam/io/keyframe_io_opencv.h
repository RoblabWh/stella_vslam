#ifndef STELLA_VSLAM_IO_KEYFRAME_IO_OPENCV_H
#define STELLA_VSLAM_IO_KEYFRAME_IO_OPENCV_H

#include "stella_vslam/io/keyframe_io_base.h"

#include <string>

namespace stella_vslam {

namespace data {
class map_database;
} // namespace data

namespace io {

class keyframe_io_opencv : public keyframe_io_base {
public:
    /**
     * Constructor
     */
    keyframe_io_opencv(const std::string &format = "png") : format_(format) {};

    /**
     * Destructor
     */
    virtual ~keyframe_io_opencv() = default;

    /**
     * Save the keyframes
     */
    bool save(const std::string& path,
              const data::map_database* const map_db) override;

private:
    /**
     * Format and file ending
     */
    std::string format_;
};

} // namespace io
} // namespace stella_vslam

#endif // STELLA_VSLAM_IO_KEYFRAME_IO_OPENCV_H
