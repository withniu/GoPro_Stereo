FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/GoPro_Stereo/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/GoPro_Stereo/msg/__init__.py"
  "../src/GoPro_Stereo/msg/_pos2d.py"
  "../src/GoPro_Stereo/msg/_pos3d.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
