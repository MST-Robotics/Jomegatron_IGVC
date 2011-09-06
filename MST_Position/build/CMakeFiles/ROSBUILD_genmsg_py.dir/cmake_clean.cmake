FILE(REMOVE_RECURSE
  "../src/MST_Position/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/MST_Position/msg/__init__.py"
  "../src/MST_Position/msg/_Target_Heading.py"
  "../src/MST_Position/msg/_target.py"
  "../src/MST_Position/msg/_inital_gps.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
