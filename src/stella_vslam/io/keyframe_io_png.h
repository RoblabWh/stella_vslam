#ifndef STELLA_VSLAM_IO_KEYFRAME_IO_PNG_H
#define STELLA_VSLAM_IO_KEYFRAME_IO_PNG_H

#include "stella_vslam/io/keyframe_io_base.h"

#include <string>

namespace stella_vslam {

namespace data {
class map_database;
} // namespace data

namespace io {

class keyframe_io_png : public keyframe_io_base {
public:
    /**
     * Constructor
     */
    keyframe_io_png() = default;

    /**
     * Destructor
     */
    virtual ~keyframe_io_png() = default;

    /**
     * Save the keyframes as PNG
     */
    bool save(const std::string& path,
              const data::map_database* const map_db) override;
};

} // namespace io
} // namespace stella_vslam

#endif // STELLA_VSLAM_IO_KEYFRAME_IO_PNG_H
