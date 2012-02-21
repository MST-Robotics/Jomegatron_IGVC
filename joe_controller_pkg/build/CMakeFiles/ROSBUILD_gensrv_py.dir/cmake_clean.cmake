FILE(REMOVE_RECURSE
  "../src/joe_controller_pkg/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/joe_controller_pkg/srv/__init__.py"
  "../src/joe_controller_pkg/srv/_GetLinkState.py"
  "../src/joe_controller_pkg/srv/_CommandVelocity.py"
  "../src/joe_controller_pkg/srv/_GetJointProperties.py"
  "../src/joe_controller_pkg/srv/_ApplyJointEffort.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
