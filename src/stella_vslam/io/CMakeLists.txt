# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_io.h
               ${CMAKE_CURRENT_SOURCE_DIR}/map_database_io_base.h
               ${CMAKE_CURRENT_SOURCE_DIR}/map_database_io_factory.h
               ${CMAKE_CURRENT_SOURCE_DIR}/map_database_io_msgpack.h
               ${CMAKE_CURRENT_SOURCE_DIR}/map_database_io_sqlite3.h
               ${CMAKE_CURRENT_SOURCE_DIR}/point_cloud_io_base.h
               ${CMAKE_CURRENT_SOURCE_DIR}/point_cloud_io_factory.h
               ${CMAKE_CURRENT_SOURCE_DIR}/point_cloud_io_ply.h
               ${CMAKE_CURRENT_SOURCE_DIR}/keyframe_io_base.h
               ${CMAKE_CURRENT_SOURCE_DIR}/keyframe_io_factory.h
               ${CMAKE_CURRENT_SOURCE_DIR}/keyframe_io_opencv.h
               ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_io.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/map_database_io_msgpack.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/map_database_io_sqlite3.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/point_cloud_io_ply.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/keyframe_io_opencv.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${STELLA_VSLAM_INCLUDE_INSTALL_DIR}/io)
