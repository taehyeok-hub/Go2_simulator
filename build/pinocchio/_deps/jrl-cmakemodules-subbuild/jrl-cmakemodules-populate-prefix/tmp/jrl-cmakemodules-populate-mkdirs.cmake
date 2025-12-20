# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-src")
  file(MAKE_DIRECTORY "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-src")
endif()
file(MAKE_DIRECTORY
  "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-build"
  "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix"
  "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/tmp"
  "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src/jrl-cmakemodules-populate-stamp"
  "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src"
  "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src/jrl-cmakemodules-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src/jrl-cmakemodules-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/pth/pth/model/Go2_simulator/build/pinocchio/_deps/jrl-cmakemodules-subbuild/jrl-cmakemodules-populate-prefix/src/jrl-cmakemodules-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
