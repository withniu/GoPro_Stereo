FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/GoPro_Stereo/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/pos2d.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_pos2d.lisp"
  "../msg_gen/lisp/pos3d.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_pos3d.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
