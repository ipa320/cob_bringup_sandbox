FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/cob_hwmonitor/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/hw_msg.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_hw_msg.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
