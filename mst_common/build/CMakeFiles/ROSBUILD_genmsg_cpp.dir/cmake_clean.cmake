FILE(REMOVE_RECURSE
  "../src/mst_common/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/mst_common/ImageFilter.h"
  "../msg_gen/cpp/include/mst_common/Filter.h"
  "../msg_gen/cpp/include/mst_common/IMU.h"
  "../msg_gen/cpp/include/mst_common/Velocity.h"
  "../msg_gen/cpp/include/mst_common/VisionFilter.h"
  "../msg_gen/cpp/include/mst_common/Raytrace.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
