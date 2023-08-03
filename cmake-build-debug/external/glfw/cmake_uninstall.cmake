
if (NOT EXISTS "/Users/fangwenyu/Desktop/loo/Spring 2023/CS 488/Snow_Simulation/cmake-build-debug/external/glfw/install_manifest.txt")
    message(FATAL_ERROR "Cannot find install manifest: \"/Users/fangwenyu/Desktop/loo/Spring 2023/CS 488/Snow_Simulation/cmake-build-debug/external/glfw/install_manifest.txt\"")
endif()

file(READ "/Users/fangwenyu/Desktop/loo/Spring 2023/CS 488/Snow_Simulation/cmake-build-debug/external/glfw/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")

foreach (file ${files})
  message(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
  if (EXISTS "$ENV{DESTDIR}${file}")
    exec_program("/Users/fangwenyu/Library/Application Support/JetBrains/Toolbox/apps/CLion/ch-0/232.8660.186/CLion.app/Contents/bin/cmake/mac/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
                 OUTPUT_VARIABLE rm_out
                 RETURN_VALUE rm_retval)
    if (NOT "${rm_retval}" STREQUAL 0)
      MESSAGE(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
    endif()
  elseif (IS_SYMLINK "$ENV{DESTDIR}${file}")
    EXEC_PROGRAM("/Users/fangwenyu/Library/Application Support/JetBrains/Toolbox/apps/CLion/ch-0/232.8660.186/CLion.app/Contents/bin/cmake/mac/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
                 OUTPUT_VARIABLE rm_out
                 RETURN_VALUE rm_retval)
    if (NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing symlink \"$ENV{DESTDIR}${file}\"")
    endif()
  else()
    message(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
  endif()
endforeach()

