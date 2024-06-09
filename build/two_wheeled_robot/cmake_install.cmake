# Install script for directory: /home/botcanh/dev_ws/src/two_wheeled_robot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/botcanh/dev_ws/src/two_wheeled_robot/install/two_wheeled_robot")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot" TYPE DIRECTORY FILES
    "/home/botcanh/dev_ws/src/two_wheeled_robot/config"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/include"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/launch"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/maps"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/meshes"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/models"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/params"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/rviz"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/scripts"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/src"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/two_wheeled_robot"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/urdf"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/worlds"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot/environment" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot/environment" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/two_wheeled_robot-1.0.0-py3.10.egg-info" TYPE DIRECTORY FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_python/two_wheeled_robot/two_wheeled_robot.egg-info/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/two_wheeled_robot" TYPE DIRECTORY FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/two_wheeled_robot/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/botcanh/dev_ws/src/two_wheeled_robot/install/two_wheeled_robot/local/lib/python3.10/dist-packages/two_wheeled_robot"
      )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/two_wheeled_robot" TYPE PROGRAM FILES
    "/home/botcanh/dev_ws/src/two_wheeled_robot/scripts/image_node.py"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/scripts/env.py"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/scripts/model5.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/two_wheeled_robot")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/two_wheeled_robot")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot/environment" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot/environment" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_index/share/ament_index/resource_index/packages/two_wheeled_robot")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot/cmake" TYPE FILE FILES
    "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_core/two_wheeled_robotConfig.cmake"
    "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/ament_cmake_core/two_wheeled_robotConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/two_wheeled_robot" TYPE FILE FILES "/home/botcanh/dev_ws/src/two_wheeled_robot/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/botcanh/dev_ws/src/two_wheeled_robot/build/two_wheeled_robot/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
