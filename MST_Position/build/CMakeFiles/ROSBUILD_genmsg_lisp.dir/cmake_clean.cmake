FILE(REMOVE_RECURSE
  "../src/MST_Position/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/inital_gps.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_inital_gps.lisp"
  "../msg_gen/lisp/Target_Heading.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Target_Heading.lisp"
  "../msg_gen/lisp/target.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_target.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
