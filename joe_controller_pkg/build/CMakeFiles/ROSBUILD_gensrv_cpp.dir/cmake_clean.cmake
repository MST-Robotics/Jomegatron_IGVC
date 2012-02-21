FILE(REMOVE_RECURSE
  "../src/joe_controller_pkg/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/joe_controller_pkg/GetLinkState.h"
  "../srv_gen/cpp/include/joe_controller_pkg/CommandVelocity.h"
  "../srv_gen/cpp/include/joe_controller_pkg/GetJointProperties.h"
  "../srv_gen/cpp/include/joe_controller_pkg/ApplyJointEffort.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
