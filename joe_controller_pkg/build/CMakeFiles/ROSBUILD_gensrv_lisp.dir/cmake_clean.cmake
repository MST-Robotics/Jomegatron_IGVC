FILE(REMOVE_RECURSE
  "../src/joe_controller_pkg/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/GetLinkState.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetLinkState.lisp"
  "../srv_gen/lisp/CommandVelocity.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_CommandVelocity.lisp"
  "../srv_gen/lisp/GetJointProperties.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_GetJointProperties.lisp"
  "../srv_gen/lisp/ApplyJointEffort.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_ApplyJointEffort.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
