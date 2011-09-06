FILE(REMOVE_RECURSE
  "../src/mst_common/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/mst_common/msg/__init__.py"
  "../src/mst_common/msg/_ImageFilter.py"
  "../src/mst_common/msg/_Filter.py"
  "../src/mst_common/msg/_IMU.py"
  "../src/mst_common/msg/_Velocity.py"
  "../src/mst_common/msg/_VisionFilter.py"
  "../src/mst_common/msg/_Raytrace.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
