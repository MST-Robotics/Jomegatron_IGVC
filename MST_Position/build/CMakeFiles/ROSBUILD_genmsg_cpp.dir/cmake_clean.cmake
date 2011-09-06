FILE(REMOVE_RECURSE
  "../src/MST_Position/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/MST_Position/inital_gps.h"
  "../msg_gen/cpp/include/MST_Position/Target_Heading.h"
  "../msg_gen/cpp/include/MST_Position/target.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
