# generated from
# ament_cmake_core/cmake/symlink_install/ament_cmake_symlink_install.cmake.in

# create empty symlink install manifest before starting install step
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/symlink_install_manifest.txt")

#
# Reimplement CMake install(DIRECTORY) command to use symlinks instead of
# copying resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_directory cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "DIRECTORY;PATTERN;PATTERN_EXCLUDE" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_directory() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/minsuje/ros2_ws/install/dsr_msgs2/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  # default pattern to include
  if(NOT ARG_PATTERN)
    set(ARG_PATTERN "*")
  endif()

  # iterate over directories
  foreach(dir ${ARG_DIRECTORY})
    # make dir an absolute path
    if(NOT IS_ABSOLUTE "${dir}")
      set(dir "${cmake_current_source_dir}/${dir}")
    endif()

    if(EXISTS "${dir}")
      # if directory has no trailing slash
      # append folder name to destination
      set(destination "${ARG_DESTINATION}")
      string(LENGTH "${dir}" length)
      math(EXPR offset "${length} - 1")
      string(SUBSTRING "${dir}" ${offset} 1 dir_last_char)
      if(NOT dir_last_char STREQUAL "/")
        get_filename_component(destination_name "${dir}" NAME)
        set(destination "${destination}/${destination_name}")
      else()
        # remove trailing slash
        string(SUBSTRING "${dir}" 0 ${offset} dir)
      endif()

      # glob recursive files
      set(relative_files "")
      foreach(pattern ${ARG_PATTERN})
        file(
          GLOB_RECURSE
          include_files
          RELATIVE "${dir}"
          "${dir}/${pattern}"
        )
        if(NOT include_files STREQUAL "")
          list(APPEND relative_files ${include_files})
        endif()
      endforeach()
      foreach(pattern ${ARG_PATTERN_EXCLUDE})
        file(
          GLOB_RECURSE
          exclude_files
          RELATIVE "${dir}"
          "${dir}/${pattern}"
        )
        if(NOT exclude_files STREQUAL "")
          list(REMOVE_ITEM relative_files ${exclude_files})
        endif()
      endforeach()
      list(SORT relative_files)

      foreach(relative_file ${relative_files})
        set(absolute_file "${dir}/${relative_file}")
        # determine link name for file including destination path
        set(symlink "${destination}/${relative_file}")

        # ensure that destination exists
        get_filename_component(symlink_dir "${symlink}" PATH)
        if(NOT EXISTS "${symlink_dir}")
          file(MAKE_DIRECTORY "${symlink_dir}")
        endif()

        _ament_cmake_symlink_install_create_symlink("${absolute_file}" "${symlink}")
      endforeach()
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_directory() can't find '${dir}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(FILES) command to use symlinks instead of copying
# resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_files cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION;RENAME" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_files() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination an absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/minsuje/ros2_ws/install/dsr_msgs2/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  if(ARG_RENAME)
    list(LENGTH ARG_FILES file_count)
    if(NOT file_count EQUAL 1)
    message(FATAL_ERROR "ament_cmake_symlink_install_files() called with "
      "RENAME argument but not with a single file")
    endif()
  endif()

  # iterate over files
  foreach(file ${ARG_FILES})
    # make file an absolute path
    if(NOT IS_ABSOLUTE "${file}")
      set(file "${cmake_current_source_dir}/${file}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      if(NOT ARG_RENAME)
        set(symlink "${ARG_DESTINATION}/${filename}")
      else()
        set(symlink "${ARG_DESTINATION}/${ARG_RENAME}")
      endif()
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_files() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(PROGRAMS) command to use symlinks instead of copying
# resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_programs cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "PROGRAMS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_programs() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination an absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/minsuje/ros2_ws/install/dsr_msgs2/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  # iterate over programs
  foreach(file ${ARG_PROGRAMS})
    # make file an absolute path
    if(NOT IS_ABSOLUTE "${file}")
      set(file "${cmake_current_source_dir}/${file}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      set(symlink "${ARG_DESTINATION}/${filename}")
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_programs() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(TARGETS) command to use symlinks instead of copying
# resources.
#
# :param TARGET_FILES: the absolute files, replacing the name of targets passed
#   in as TARGETS
# :type TARGET_FILES: list of files
# :param ARGN: the same arguments as the CMake install command except that
#   keywords identifying the kind of type and the DESTINATION keyword must be
#   joined with an underscore, e.g. ARCHIVE_DESTINATION.
# :type ARGN: various
#
function(ament_cmake_symlink_install_targets)
  cmake_parse_arguments(ARG "OPTIONAL" "ARCHIVE_DESTINATION;DESTINATION;LIBRARY_DESTINATION;RUNTIME_DESTINATION"
    "TARGETS;TARGET_FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_targets() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # iterate over target files
  foreach(file ${ARG_TARGET_FILES})
    if(NOT IS_ABSOLUTE "${file}")
      message(FATAL_ERROR "ament_cmake_symlink_install_targets() target file "
        "'${file}' must be an absolute path")
    endif()

    # determine destination of file based on extension
    set(destination "")
    get_filename_component(fileext "${file}" EXT)
    if(fileext STREQUAL ".a" OR fileext STREQUAL ".lib")
      set(destination "${ARG_ARCHIVE_DESTINATION}")
    elseif(fileext STREQUAL ".dylib" OR fileext MATCHES "\\.so(\\.[0-9]+)?(\\.[0-9]+)?(\\.[0-9]+)?$")
      set(destination "${ARG_LIBRARY_DESTINATION}")
    elseif(fileext STREQUAL "" OR fileext STREQUAL ".dll" OR fileext STREQUAL ".exe")
      set(destination "${ARG_RUNTIME_DESTINATION}")
    endif()
    if(destination STREQUAL "")
      set(destination "${ARG_DESTINATION}")
    endif()

    # make destination an absolute path and ensure that it exists
    if(NOT IS_ABSOLUTE "${destination}")
      set(destination "/home/minsuje/ros2_ws/install/dsr_msgs2/${destination}")
    endif()
    if(NOT EXISTS "${destination}")
      file(MAKE_DIRECTORY "${destination}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      set(symlink "${destination}/${filename}")
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_targets() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

function(_ament_cmake_symlink_install_create_symlink absolute_file symlink)
  # register symlink for being removed during install step
  file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/symlink_install_manifest.txt"
    "${symlink}\n")

  # avoid any work if correct symlink is already in place
  if(EXISTS "${symlink}" AND IS_SYMLINK "${symlink}")
    get_filename_component(destination "${symlink}" REALPATH)
    get_filename_component(real_absolute_file "${absolute_file}" REALPATH)
    if(destination STREQUAL real_absolute_file)
      message(STATUS "Up-to-date symlink: ${symlink}")
      return()
    endif()
  endif()

  message(STATUS "Symlinking: ${symlink}")
  if(EXISTS "${symlink}" OR IS_SYMLINK "${symlink}")
    file(REMOVE "${symlink}")
  endif()

  execute_process(
    COMMAND "/usr/bin/cmake" "-E" "create_symlink"
      "${absolute_file}"
      "${symlink}"
  )
  # the CMake command does not provide a return code so check manually
  if(NOT EXISTS "${symlink}" OR NOT IS_SYMLINK "${symlink}")
    get_filename_component(destination "${symlink}" REALPATH)
    message(FATAL_ERROR
      "Could not create symlink '${symlink}' pointing to '${absolute_file}'")
  endif()
endfunction()

# end of template

message(STATUS "Execute custom install script")

# begin of custom install code

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/dsr_msgs2" "DESTINATION" "share/ament_index/resource_index/rosidl_interfaces")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/dsr_msgs2" "DESTINATION" "share/ament_index/resource_index/rosidl_interfaces")

# install(DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_generator_c/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN" "*.h")
ament_cmake_symlink_install_directory("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_generator_c/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN" "*.h")

# install(FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh" "DESTINATION" "share/dsr_msgs2/environment")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh" "DESTINATION" "share/dsr_msgs2/environment")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/library_path.dsv" "DESTINATION" "share/dsr_msgs2/environment")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/library_path.dsv" "DESTINATION" "share/dsr_msgs2/environment")

# install(DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_typesupport_fastrtps_c/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN_EXCLUDE" "*.cpp")
ament_cmake_symlink_install_directory("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_typesupport_fastrtps_c/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN_EXCLUDE" "*.cpp")

# install(DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_generator_cpp/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN" "*.hpp")
ament_cmake_symlink_install_directory("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_generator_cpp/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN" "*.hpp")

# install(DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_typesupport_fastrtps_cpp/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN_EXCLUDE" "*.cpp")
ament_cmake_symlink_install_directory("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_typesupport_fastrtps_cpp/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN_EXCLUDE" "*.cpp")

# install(DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_typesupport_introspection_c/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN" "*.h")
ament_cmake_symlink_install_directory("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_typesupport_introspection_c/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN" "*.h")

# install(DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_typesupport_introspection_cpp/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN" "*.hpp")
ament_cmake_symlink_install_directory("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_typesupport_introspection_cpp/dsr_msgs2/" "DESTINATION" "include/dsr_msgs2/dsr_msgs2" "PATTERN" "*.hpp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/pythonpath.sh" "DESTINATION" "share/dsr_msgs2/environment")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/pythonpath.sh" "DESTINATION" "share/dsr_msgs2/environment")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/pythonpath.dsv" "DESTINATION" "share/dsr_msgs2/environment")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/pythonpath.dsv" "DESTINATION" "share/dsr_msgs2/environment")

# install(DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_python/dsr_msgs2/dsr_msgs2.egg-info/" "DESTINATION" "local/lib/python3.10/dist-packages/dsr_msgs2-1.1.0-py3.10.egg-info")
ament_cmake_symlink_install_directory("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_python/dsr_msgs2/dsr_msgs2.egg-info/" "DESTINATION" "local/lib/python3.10/dist-packages/dsr_msgs2-1.1.0-py3.10.egg-info")

# install(DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_generator_py/dsr_msgs2/" "DESTINATION" "local/lib/python3.10/dist-packages/dsr_msgs2" "PATTERN_EXCLUDE" "*.pyc" "PATTERN_EXCLUDE" "__pycache__")
ament_cmake_symlink_install_directory("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" DIRECTORY "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_generator_py/dsr_msgs2/" "DESTINATION" "local/lib/python3.10/dist-packages/dsr_msgs2" "PATTERN_EXCLUDE" "*.pyc" "PATTERN_EXCLUDE" "__pycache__")

# install("TARGETS" "dsr_msgs2__rosidl_typesupport_fastrtps_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/dsr_msgs2")
include("/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_symlink_install_targets_0_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install("TARGETS" "dsr_msgs2__rosidl_typesupport_introspection_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/dsr_msgs2")
include("/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_symlink_install_targets_1_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install("TARGETS" "dsr_msgs2__rosidl_typesupport_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/dsr_msgs2")
include("/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_symlink_install_targets_2_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/LogAlarm.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/LogAlarm.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ModbusState.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ModbusState.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/RobotError.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/RobotError.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/RobotState.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/RobotState.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/RobotStop.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/RobotStop.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/JogMultiAxis.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/JogMultiAxis.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/AlterMotionStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/AlterMotionStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ServojStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ServojStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ServolStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ServolStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/SpeedjStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/SpeedjStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/SpeedlStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/SpeedlStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/RobotStateRt.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/RobotStateRt.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ServojRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ServojRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ServolRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/ServolRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/SpeedjRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/SpeedjRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/SpeedlRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/SpeedlRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/TorqueRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/msg/TorqueRtStream.idl" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRobotMode.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRobotMode.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRobotMode.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRobotMode.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRobotSystem.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRobotSystem.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRobotSystem.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRobotSystem.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRobotSpeedMode.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRobotSpeedMode.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRobotSpeedMode.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRobotSpeedMode.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentPose.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentPose.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetSafeStopResetType.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetSafeStopResetType.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetLastAlarm.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetLastAlarm.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRobotState.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRobotState.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ServoOff.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ServoOff.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRobotControl.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRobotControl.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ChangeCollisionSensitivity.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ChangeCollisionSensitivity.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetSafetyMode.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetSafetyMode.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveJoint.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveJoint.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveLine.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveLine.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveJointx.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveJointx.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveCircle.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveCircle.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveSplineJoint.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveSplineJoint.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveSplineTask.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveSplineTask.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveBlending.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveBlending.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveSpiral.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveSpiral.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MovePeriodic.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MovePeriodic.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveWait.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveWait.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Jog.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Jog.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/JogMulti.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/JogMulti.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveStop.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveStop.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MovePause.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MovePause.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveResume.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveResume.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Trans.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Trans.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Fkin.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Fkin.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Ikin.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Ikin.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRefCoord.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRefCoord.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveHome.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/MoveHome.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckMotion.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckMotion.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ChangeOperationSpeed.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ChangeOperationSpeed.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/EnableAlterMotion.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/EnableAlterMotion.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/AlterMotion.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/AlterMotion.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DisableAlterMotion.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DisableAlterMotion.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetSingularityHandling.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetSingularityHandling.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetControlMode.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetControlMode.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetControlSpace.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetControlSpace.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentPosj.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentPosj.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDesiredPosj.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDesiredPosj.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentVelj.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentVelj.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDesiredVelj.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDesiredVelj.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentPosx.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentPosx.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentToolFlangePosx.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentToolFlangePosx.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentVelx.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentVelx.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDesiredPosx.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDesiredPosx.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDesiredVelx.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDesiredVelx.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentSolutionSpace.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentSolutionSpace.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentRotm.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentRotm.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetJointTorque.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetJointTorque.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetExternalTorque.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetExternalTorque.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetToolForce.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetToolForce.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetSolutionSpace.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetSolutionSpace.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetOrientationError.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetOrientationError.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ParallelAxis1.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ParallelAxis1.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ParallelAxis2.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ParallelAxis2.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/AlignAxis1.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/AlignAxis1.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/AlignAxis2.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/AlignAxis2.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/IsDoneBoltTightening.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/IsDoneBoltTightening.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ReleaseComplianceCtrl.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ReleaseComplianceCtrl.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/TaskComplianceCtrl.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/TaskComplianceCtrl.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetStiffnessx.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetStiffnessx.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CalcCoord.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CalcCoord.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetUserCartCoord1.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetUserCartCoord1.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetUserCartCoord2.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetUserCartCoord2.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetUserCartCoord3.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetUserCartCoord3.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/OverwriteUserCartCoord.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/OverwriteUserCartCoord.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetUserCartCoord.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetUserCartCoord.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetDesiredForce.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetDesiredForce.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ReleaseForce.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ReleaseForce.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckPositionCondition.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckPositionCondition.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckForceCondition.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckForceCondition.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckOrientationCondition1.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckOrientationCondition1.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckOrientationCondition2.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CheckOrientationCondition2.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CoordTransform.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/CoordTransform.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetWorkpieceWeight.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetWorkpieceWeight.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ResetWorkpieceWeight.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ResetWorkpieceWeight.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigCreateTool.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigCreateTool.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigDeleteTool.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigDeleteTool.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCurrentTool.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCurrentTool.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentTool.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentTool.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetToolShape.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetToolShape.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigCreateTcp.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigCreateTcp.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigDeleteTcp.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigDeleteTcp.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCurrentTcp.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCurrentTcp.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentTcp.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCurrentTcp.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetToolDigitalOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetToolDigitalOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetToolDigitalOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetToolDigitalOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetToolDigitalInput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetToolDigitalInput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCtrlBoxDigitalOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCtrlBoxDigitalOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCtrlBoxDigitalOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCtrlBoxDigitalOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCtrlBoxDigitalInput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCtrlBoxDigitalInput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCtrlBoxAnalogInputType.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCtrlBoxAnalogInputType.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCtrlBoxAnalogOutputType.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCtrlBoxAnalogOutputType.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCtrlBoxAnalogOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetCtrlBoxAnalogOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCtrlBoxAnalogInput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetCtrlBoxAnalogInput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigCreateModbus.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigCreateModbus.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigDeleteModbus.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConfigDeleteModbus.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetModbusOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetModbusOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetModbusInput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetModbusInput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DrlStart.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DrlStart.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DrlStop.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DrlStop.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DrlPause.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DrlPause.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DrlResume.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DrlResume.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDrlState.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetDrlState.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Robotiq2FClose.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Robotiq2FClose.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Robotiq2FOpen.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Robotiq2FOpen.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Robotiq2FMove.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/Robotiq2FMove.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SerialSendData.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SerialSendData.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConnectRtControl.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ConnectRtControl.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DisconnectRtControl.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/DisconnectRtControl.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRtControlInputDataList.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRtControlInputDataList.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRtControlInputVersionList.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRtControlInputVersionList.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRtControlOutputDataList.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRtControlOutputDataList.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRtControlOutputVersionList.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/GetRtControlOutputVersionList.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ReadDataRt.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/ReadDataRt.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetAccjRt.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetAccjRt.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetAccxRt.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetAccxRt.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRtControlInput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRtControlInput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRtControlOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetRtControlOutput.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetVeljRt.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetVeljRt.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetVelxRt.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/SetVelxRt.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/StartRtControl.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/StartRtControl.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/StopRtControl.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/StopRtControl.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/WriteDataRt.idl" "DESTINATION" "share/dsr_msgs2/srv")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_adapter/dsr_msgs2/srv/WriteDataRt.idl" "DESTINATION" "share/dsr_msgs2/srv")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/LogAlarm.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/LogAlarm.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ModbusState.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ModbusState.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/RobotError.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/RobotError.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/RobotState.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/RobotState.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/RobotStop.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/RobotStop.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/JogMultiAxis.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/JogMultiAxis.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/AlterMotionStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/AlterMotionStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ServojStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ServojStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ServolStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ServolStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/SpeedjStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/SpeedjStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/SpeedlStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/SpeedlStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/RobotStateRt.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/RobotStateRt.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ServojRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ServojRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ServolRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/ServolRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/SpeedjRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/SpeedjRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/SpeedlRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/SpeedlRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/TorqueRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/msg/TorqueRtStream.msg" "DESTINATION" "share/dsr_msgs2/msg")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetRobotMode.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetRobotMode.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetRobotMode.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetRobotMode.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetRobotSystem.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetRobotSystem.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotSystem_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotSystem_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotSystem_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotSystem_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetRobotSystem.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetRobotSystem.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotSystem_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotSystem_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotSystem_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotSystem_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetRobotSpeedMode.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetRobotSpeedMode.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotSpeedMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotSpeedMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotSpeedMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotSpeedMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetRobotSpeedMode.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetRobotSpeedMode.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotSpeedMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotSpeedMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotSpeedMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotSpeedMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetCurrentPose.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetCurrentPose.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetCurrentPose_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetCurrentPose_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetCurrentPose_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetCurrentPose_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetSafeStopResetType.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetSafeStopResetType.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetSafeStopResetType_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetSafeStopResetType_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetSafeStopResetType_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetSafeStopResetType_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetLastAlarm.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetLastAlarm.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetLastAlarm_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetLastAlarm_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetLastAlarm_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetLastAlarm_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetRobotState.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/GetRobotState.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotState_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotState_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotState_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/GetRobotState_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/ServoOff.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/ServoOff.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/ServoOff_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/ServoOff_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/ServoOff_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/ServoOff_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetRobotControl.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetRobotControl.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotControl_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotControl_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotControl_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetRobotControl_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/ChangeCollisionSensitivity.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/ChangeCollisionSensitivity.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/ChangeCollisionSensitivity_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/ChangeCollisionSensitivity_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/ChangeCollisionSensitivity_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/ChangeCollisionSensitivity_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetSafetyMode.srv" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/system/SetSafetyMode.srv" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetSafetyMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetSafetyMode_Request.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetSafetyMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/system/SetSafetyMode_Response.msg" "DESTINATION" "share/dsr_msgs2/system")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveJoint.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveJoint.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveJoint_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveJoint_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveJoint_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveJoint_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveLine.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveLine.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveLine_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveLine_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveLine_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveLine_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveJointx.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveJointx.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveJointx_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveJointx_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveJointx_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveJointx_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveCircle.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveCircle.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveCircle_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveCircle_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveCircle_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveCircle_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveSplineJoint.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveSplineJoint.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSplineJoint_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSplineJoint_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSplineJoint_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSplineJoint_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveSplineTask.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveSplineTask.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSplineTask_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSplineTask_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSplineTask_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSplineTask_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveBlending.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveBlending.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveBlending_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveBlending_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveBlending_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveBlending_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveSpiral.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveSpiral.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSpiral_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSpiral_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSpiral_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveSpiral_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MovePeriodic.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MovePeriodic.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MovePeriodic_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MovePeriodic_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MovePeriodic_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MovePeriodic_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveWait.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveWait.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveWait_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveWait_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveWait_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveWait_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/Jog.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/Jog.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Jog_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Jog_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Jog_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Jog_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/JogMulti.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/JogMulti.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/JogMulti_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/JogMulti_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/JogMulti_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/JogMulti_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveStop.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveStop.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveStop_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveStop_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveStop_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveStop_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MovePause.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MovePause.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MovePause_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MovePause_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MovePause_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MovePause_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveResume.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveResume.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveResume_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveResume_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveResume_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveResume_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/Trans.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/Trans.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Trans_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Trans_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Trans_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Trans_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/Fkin.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/Fkin.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Fkin_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Fkin_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Fkin_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Fkin_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/Ikin.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/Ikin.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Ikin_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Ikin_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Ikin_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/Ikin_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/SetRefCoord.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/SetRefCoord.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/SetRefCoord_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/SetRefCoord_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/SetRefCoord_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/SetRefCoord_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveHome.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/MoveHome.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveHome_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveHome_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveHome_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/MoveHome_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/CheckMotion.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/CheckMotion.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/CheckMotion_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/CheckMotion_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/CheckMotion_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/CheckMotion_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/ChangeOperationSpeed.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/ChangeOperationSpeed.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/ChangeOperationSpeed_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/ChangeOperationSpeed_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/ChangeOperationSpeed_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/ChangeOperationSpeed_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/EnableAlterMotion.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/EnableAlterMotion.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/EnableAlterMotion_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/EnableAlterMotion_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/EnableAlterMotion_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/EnableAlterMotion_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/AlterMotion.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/AlterMotion.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/AlterMotion_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/AlterMotion_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/AlterMotion_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/AlterMotion_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/DisableAlterMotion.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/DisableAlterMotion.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/DisableAlterMotion_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/DisableAlterMotion_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/DisableAlterMotion_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/DisableAlterMotion_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/SetSingularityHandling.srv" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/motion/SetSingularityHandling.srv" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/SetSingularityHandling_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/SetSingularityHandling_Request.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/SetSingularityHandling_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/motion/SetSingularityHandling_Response.msg" "DESTINATION" "share/dsr_msgs2/motion")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetControlMode.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetControlMode.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetControlMode_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetControlMode_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetControlMode_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetControlMode_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetControlSpace.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetControlSpace.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetControlSpace_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetControlSpace_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetControlSpace_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetControlSpace_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentPosj.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentPosj.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentPosj_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentPosj_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentPosj_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentPosj_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetDesiredPosj.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetDesiredPosj.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredPosj_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredPosj_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredPosj_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredPosj_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentVelj.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentVelj.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentVelj_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentVelj_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentVelj_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentVelj_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetDesiredVelj.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetDesiredVelj.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredVelj_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredVelj_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredVelj_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredVelj_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentPosx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentPosx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentPosx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentPosx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentPosx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentPosx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentToolFlangePosx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentToolFlangePosx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentToolFlangePosx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentToolFlangePosx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentToolFlangePosx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentToolFlangePosx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentVelx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentVelx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentVelx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentVelx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentVelx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentVelx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetDesiredPosx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetDesiredPosx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredPosx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredPosx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredPosx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredPosx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetDesiredVelx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetDesiredVelx.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredVelx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredVelx_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredVelx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetDesiredVelx_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentSolutionSpace.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentSolutionSpace.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentSolutionSpace_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentSolutionSpace_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentSolutionSpace_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentSolutionSpace_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentRotm.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetCurrentRotm.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentRotm_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentRotm_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentRotm_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetCurrentRotm_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetJointTorque.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetJointTorque.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetJointTorque_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetJointTorque_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetJointTorque_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetJointTorque_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetExternalTorque.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetExternalTorque.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetExternalTorque_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetExternalTorque_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetExternalTorque_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetExternalTorque_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetToolForce.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetToolForce.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetToolForce_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetToolForce_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetToolForce_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetToolForce_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetSolutionSpace.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetSolutionSpace.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetSolutionSpace_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetSolutionSpace_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetSolutionSpace_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetSolutionSpace_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetOrientationError.srv" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/aux_control/GetOrientationError.srv" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetOrientationError_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetOrientationError_Request.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetOrientationError_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/aux_control/GetOrientationError_Response.msg" "DESTINATION" "share/dsr_msgs2/aux_control")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ParallelAxis1.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ParallelAxis1.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ParallelAxis1_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ParallelAxis1_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ParallelAxis1_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ParallelAxis1_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ParallelAxis2.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ParallelAxis2.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ParallelAxis2_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ParallelAxis2_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ParallelAxis2_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ParallelAxis2_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/AlignAxis1.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/AlignAxis1.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/AlignAxis1_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/AlignAxis1_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/AlignAxis1_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/AlignAxis1_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/AlignAxis2.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/AlignAxis2.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/AlignAxis2_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/AlignAxis2_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/AlignAxis2_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/AlignAxis2_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/IsDoneBoltTightening.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/IsDoneBoltTightening.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/IsDoneBoltTightening_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/IsDoneBoltTightening_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/IsDoneBoltTightening_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/IsDoneBoltTightening_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ReleaseComplianceCtrl.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ReleaseComplianceCtrl.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ReleaseComplianceCtrl_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ReleaseComplianceCtrl_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ReleaseComplianceCtrl_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ReleaseComplianceCtrl_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/TaskComplianceCtrl.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/TaskComplianceCtrl.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/TaskComplianceCtrl_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/TaskComplianceCtrl_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/TaskComplianceCtrl_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/TaskComplianceCtrl_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetStiffnessx.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetStiffnessx.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetStiffnessx_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetStiffnessx_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetStiffnessx_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetStiffnessx_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CalcCoord.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CalcCoord.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CalcCoord_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CalcCoord_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CalcCoord_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CalcCoord_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetUserCartCoord1.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetUserCartCoord1.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord1_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord1_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord1_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord1_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetUserCartCoord2.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetUserCartCoord2.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord2_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord2_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord2_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord2_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetUserCartCoord3.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetUserCartCoord3.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord3_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord3_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord3_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetUserCartCoord3_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/OverwriteUserCartCoord.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/OverwriteUserCartCoord.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/OverwriteUserCartCoord_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/OverwriteUserCartCoord_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/OverwriteUserCartCoord_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/OverwriteUserCartCoord_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/GetUserCartCoord.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/GetUserCartCoord.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/GetUserCartCoord_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/GetUserCartCoord_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/GetUserCartCoord_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/GetUserCartCoord_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetDesiredForce.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/SetDesiredForce.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetDesiredForce_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetDesiredForce_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetDesiredForce_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/SetDesiredForce_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ReleaseForce.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ReleaseForce.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ReleaseForce_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ReleaseForce_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ReleaseForce_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ReleaseForce_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CheckPositionCondition.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CheckPositionCondition.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckPositionCondition_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckPositionCondition_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckPositionCondition_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckPositionCondition_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CheckForceCondition.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CheckForceCondition.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckForceCondition_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckForceCondition_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckForceCondition_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckForceCondition_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CheckOrientationCondition1.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CheckOrientationCondition1.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckOrientationCondition1_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckOrientationCondition1_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckOrientationCondition1_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckOrientationCondition1_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CheckOrientationCondition2.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CheckOrientationCondition2.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckOrientationCondition2_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckOrientationCondition2_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckOrientationCondition2_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CheckOrientationCondition2_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CoordTransform.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/CoordTransform.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CoordTransform_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CoordTransform_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CoordTransform_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/CoordTransform_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/GetWorkpieceWeight.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/GetWorkpieceWeight.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/GetWorkpieceWeight_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/GetWorkpieceWeight_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/GetWorkpieceWeight_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/GetWorkpieceWeight_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ResetWorkpieceWeight.srv" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/force/ResetWorkpieceWeight.srv" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ResetWorkpieceWeight_Request.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ResetWorkpieceWeight_Request.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ResetWorkpieceWeight_Response.msg" "DESTINATION" "share/dsr_msgs2/force")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/force/ResetWorkpieceWeight_Response.msg" "DESTINATION" "share/dsr_msgs2/force")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/ConfigCreateTool.srv" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/ConfigCreateTool.srv" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/ConfigCreateTool_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/ConfigCreateTool_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/ConfigCreateTool_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/ConfigCreateTool_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/ConfigDeleteTool.srv" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/ConfigDeleteTool.srv" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/ConfigDeleteTool_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/ConfigDeleteTool_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/ConfigDeleteTool_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/ConfigDeleteTool_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/SetCurrentTool.srv" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/SetCurrentTool.srv" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/SetCurrentTool_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/SetCurrentTool_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/SetCurrentTool_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/SetCurrentTool_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/GetCurrentTool.srv" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/GetCurrentTool.srv" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/GetCurrentTool_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/GetCurrentTool_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/GetCurrentTool_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/GetCurrentTool_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/SetToolShape.srv" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tool/SetToolShape.srv" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/SetToolShape_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/SetToolShape_Request.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/SetToolShape_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tool/SetToolShape_Response.msg" "DESTINATION" "share/dsr_msgs2/tool")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tcp/ConfigCreateTcp.srv" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tcp/ConfigCreateTcp.srv" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/ConfigCreateTcp_Request.msg" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/ConfigCreateTcp_Request.msg" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/ConfigCreateTcp_Response.msg" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/ConfigCreateTcp_Response.msg" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tcp/ConfigDeleteTcp.srv" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tcp/ConfigDeleteTcp.srv" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/ConfigDeleteTcp_Request.msg" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/ConfigDeleteTcp_Request.msg" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/ConfigDeleteTcp_Response.msg" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/ConfigDeleteTcp_Response.msg" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tcp/SetCurrentTcp.srv" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tcp/SetCurrentTcp.srv" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/SetCurrentTcp_Request.msg" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/SetCurrentTcp_Request.msg" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/SetCurrentTcp_Response.msg" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/SetCurrentTcp_Response.msg" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tcp/GetCurrentTcp.srv" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/tcp/GetCurrentTcp.srv" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/GetCurrentTcp_Request.msg" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/GetCurrentTcp_Request.msg" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/GetCurrentTcp_Response.msg" "DESTINATION" "share/dsr_msgs2/tcp")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/tcp/GetCurrentTcp_Response.msg" "DESTINATION" "share/dsr_msgs2/tcp")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetToolDigitalOutput.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetToolDigitalOutput.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetToolDigitalOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetToolDigitalOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetToolDigitalOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetToolDigitalOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetToolDigitalOutput.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetToolDigitalOutput.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetToolDigitalOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetToolDigitalOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetToolDigitalOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetToolDigitalOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetToolDigitalInput.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetToolDigitalInput.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetToolDigitalInput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetToolDigitalInput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetToolDigitalInput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetToolDigitalInput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetCtrlBoxDigitalOutput.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetCtrlBoxDigitalOutput.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxDigitalOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxDigitalOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxDigitalOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxDigitalOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetCtrlBoxDigitalOutput.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetCtrlBoxDigitalOutput.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxDigitalOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxDigitalOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxDigitalOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxDigitalOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetCtrlBoxDigitalInput.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetCtrlBoxDigitalInput.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxDigitalInput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxDigitalInput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxDigitalInput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxDigitalInput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetCtrlBoxAnalogInputType.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetCtrlBoxAnalogInputType.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogInputType_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogInputType_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogInputType_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogInputType_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetCtrlBoxAnalogOutputType.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetCtrlBoxAnalogOutputType.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogOutputType_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogOutputType_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogOutputType_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogOutputType_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetCtrlBoxAnalogOutput.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/SetCtrlBoxAnalogOutput.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/SetCtrlBoxAnalogOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetCtrlBoxAnalogInput.srv" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/io/GetCtrlBoxAnalogInput.srv" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxAnalogInput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxAnalogInput_Request.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxAnalogInput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/io/GetCtrlBoxAnalogInput_Response.msg" "DESTINATION" "share/dsr_msgs2/io")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/modbus/ConfigCreateModbus.srv" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/modbus/ConfigCreateModbus.srv" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/ConfigCreateModbus_Request.msg" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/ConfigCreateModbus_Request.msg" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/ConfigCreateModbus_Response.msg" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/ConfigCreateModbus_Response.msg" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/modbus/ConfigDeleteModbus.srv" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/modbus/ConfigDeleteModbus.srv" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/ConfigDeleteModbus_Request.msg" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/ConfigDeleteModbus_Request.msg" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/ConfigDeleteModbus_Response.msg" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/ConfigDeleteModbus_Response.msg" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/modbus/SetModbusOutput.srv" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/modbus/SetModbusOutput.srv" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/SetModbusOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/SetModbusOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/SetModbusOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/SetModbusOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/modbus/GetModbusInput.srv" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/modbus/GetModbusInput.srv" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/GetModbusInput_Request.msg" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/GetModbusInput_Request.msg" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/GetModbusInput_Response.msg" "DESTINATION" "share/dsr_msgs2/modbus")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/modbus/GetModbusInput_Response.msg" "DESTINATION" "share/dsr_msgs2/modbus")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/DrlStart.srv" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/DrlStart.srv" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlStart_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlStart_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlStart_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlStart_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/DrlStop.srv" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/DrlStop.srv" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlStop_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlStop_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlStop_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlStop_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/DrlPause.srv" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/DrlPause.srv" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlPause_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlPause_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlPause_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlPause_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/DrlResume.srv" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/DrlResume.srv" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlResume_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlResume_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlResume_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/DrlResume_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/GetDrlState.srv" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/drl/GetDrlState.srv" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/GetDrlState_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/GetDrlState_Request.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/GetDrlState_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/drl/GetDrlState_Response.msg" "DESTINATION" "share/dsr_msgs2/drl")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/gripper/Robotiq2FClose.srv" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/gripper/Robotiq2FClose.srv" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FClose_Request.msg" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FClose_Request.msg" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FClose_Response.msg" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FClose_Response.msg" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/gripper/Robotiq2FOpen.srv" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/gripper/Robotiq2FOpen.srv" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FOpen_Request.msg" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FOpen_Request.msg" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FOpen_Response.msg" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FOpen_Response.msg" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/gripper/Robotiq2FMove.srv" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/gripper/Robotiq2FMove.srv" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FMove_Request.msg" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FMove_Request.msg" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FMove_Response.msg" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/Robotiq2FMove_Response.msg" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/gripper/SerialSendData.srv" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/gripper/SerialSendData.srv" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/SerialSendData_Request.msg" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/SerialSendData_Request.msg" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/SerialSendData_Response.msg" "DESTINATION" "share/dsr_msgs2/gripper")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/gripper/SerialSendData_Response.msg" "DESTINATION" "share/dsr_msgs2/gripper")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/ConnectRtControl.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/ConnectRtControl.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/ConnectRtControl_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/ConnectRtControl_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/ConnectRtControl_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/ConnectRtControl_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/DisconnectRtControl.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/DisconnectRtControl.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/DisconnectRtControl_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/DisconnectRtControl_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/DisconnectRtControl_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/DisconnectRtControl_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/GetRtControlInputDataList.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/GetRtControlInputDataList.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlInputDataList_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlInputDataList_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlInputDataList_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlInputDataList_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/GetRtControlInputVersionList.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/GetRtControlInputVersionList.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlInputVersionList_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlInputVersionList_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlInputVersionList_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlInputVersionList_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/GetRtControlOutputDataList.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/GetRtControlOutputDataList.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlOutputDataList_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlOutputDataList_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlOutputDataList_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlOutputDataList_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/GetRtControlOutputVersionList.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/GetRtControlOutputVersionList.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlOutputVersionList_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlOutputVersionList_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlOutputVersionList_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/GetRtControlOutputVersionList_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/ReadDataRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/ReadDataRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/ReadDataRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/ReadDataRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/ReadDataRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/ReadDataRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetAccjRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetAccjRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetAccjRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetAccjRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetAccjRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetAccjRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetAccxRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetAccxRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetAccxRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetAccxRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetAccxRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetAccxRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetRtControlInput.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetRtControlInput.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetRtControlInput_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetRtControlInput_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetRtControlInput_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetRtControlInput_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetRtControlOutput.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetRtControlOutput.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetRtControlOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetRtControlOutput_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetRtControlOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetRtControlOutput_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetVeljRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetVeljRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetVeljRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetVeljRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetVeljRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetVeljRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetVelxRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/SetVelxRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetVelxRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetVelxRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetVelxRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/SetVelxRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/StartRtControl.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/StartRtControl.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/StartRtControl_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/StartRtControl_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/StartRtControl_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/StartRtControl_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/StopRtControl.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/StopRtControl.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/StopRtControl_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/StopRtControl_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/StopRtControl_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/StopRtControl_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/WriteDataRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/srv/realtime/WriteDataRt.srv" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/WriteDataRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/WriteDataRt_Request.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/WriteDataRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/srv/realtime/WriteDataRt_Response.msg" "DESTINATION" "share/dsr_msgs2/realtime")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/dsr_msgs2" "DESTINATION" "share/ament_index/resource_index/package_run_dependencies")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/dsr_msgs2" "DESTINATION" "share/ament_index/resource_index/package_run_dependencies")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/dsr_msgs2" "DESTINATION" "share/ament_index/resource_index/parent_prefix_path")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/dsr_msgs2" "DESTINATION" "share/ament_index/resource_index/parent_prefix_path")

# install(FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh" "DESTINATION" "share/dsr_msgs2/environment")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh" "DESTINATION" "share/dsr_msgs2/environment")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/ament_prefix_path.dsv" "DESTINATION" "share/dsr_msgs2/environment")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/ament_prefix_path.dsv" "DESTINATION" "share/dsr_msgs2/environment")

# install(FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh" "DESTINATION" "share/dsr_msgs2/environment")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh" "DESTINATION" "share/dsr_msgs2/environment")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/path.dsv" "DESTINATION" "share/dsr_msgs2/environment")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/path.dsv" "DESTINATION" "share/dsr_msgs2/environment")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/local_setup.bash" "DESTINATION" "share/dsr_msgs2")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/local_setup.bash" "DESTINATION" "share/dsr_msgs2")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/local_setup.sh" "DESTINATION" "share/dsr_msgs2")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/local_setup.sh" "DESTINATION" "share/dsr_msgs2")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/local_setup.zsh" "DESTINATION" "share/dsr_msgs2")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/local_setup.zsh" "DESTINATION" "share/dsr_msgs2")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/local_setup.dsv" "DESTINATION" "share/dsr_msgs2")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/local_setup.dsv" "DESTINATION" "share/dsr_msgs2")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/package.dsv" "DESTINATION" "share/dsr_msgs2")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_environment_hooks/package.dsv" "DESTINATION" "share/dsr_msgs2")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_index/share/ament_index/resource_index/packages/dsr_msgs2" "DESTINATION" "share/ament_index/resource_index/packages")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_index/share/ament_index/resource_index/packages/dsr_msgs2" "DESTINATION" "share/ament_index/resource_index/packages")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/rosidl_cmake-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/rosidl_cmake-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake" "DESTINATION" "share/dsr_msgs2/cmake")

# install(FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_core/dsr_msgs2Config.cmake" "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_core/dsr_msgs2Config-version.cmake" "DESTINATION" "share/dsr_msgs2/cmake")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_core/dsr_msgs2Config.cmake" "/home/minsuje/ros2_ws/build/dsr_msgs2/ament_cmake_core/dsr_msgs2Config-version.cmake" "DESTINATION" "share/dsr_msgs2/cmake")

# install(FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/package.xml" "DESTINATION" "share/dsr_msgs2")
ament_cmake_symlink_install_files("/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2" FILES "/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_msgs2/package.xml" "DESTINATION" "share/dsr_msgs2")
