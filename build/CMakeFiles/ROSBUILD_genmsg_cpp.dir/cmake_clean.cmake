FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/GoPro_Stereo/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/GoPro_Stereo/pos2d.h"
  "../msg_gen/cpp/include/GoPro_Stereo/pos3d.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
