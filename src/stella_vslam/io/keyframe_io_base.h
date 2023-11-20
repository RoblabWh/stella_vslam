#ifndef STELLA_VSLAM_IO_KEYFRAME_IO_BASE_H
#define STELLA_VSLAM_IO_KEYFRAME_IO_BASE_H

#include <string>

namespace stella_vslam {

namespace data {
class map_database;
} // namespace data

namespace io {

class keyframe_io_base {
public:
    /**
     * Save the keyframe
     */
    virtual bool save(const std::string& path,
                      const data::map_database* const map_db)
        = 0;
};

} // namespace io
} // namespace stella_vslam

#endif // STELLA_VSLAM_IO_KEYFRAME_IO_BASE_H
