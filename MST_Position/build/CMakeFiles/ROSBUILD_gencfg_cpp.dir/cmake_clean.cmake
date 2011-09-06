FILE(REMOVE_RECURSE
  "../src/MST_Position/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/MST_Position/Position_ParamsConfig.h"
  "../docs/Position_ParamsConfig.dox"
  "../docs/Position_ParamsConfig-usage.dox"
  "../src/MST_Position/cfg/Position_ParamsConfig.py"
  "../docs/Position_ParamsConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
