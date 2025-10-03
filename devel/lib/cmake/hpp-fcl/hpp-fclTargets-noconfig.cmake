#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hpp-fcl::hpp-fcl" for configuration ""
set_property(TARGET hpp-fcl::hpp-fcl APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(hpp-fcl::hpp-fcl PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "/home/pth/pth/model/Go2_simulator/devel/lib/libhpp-fcl.so"
  IMPORTED_SONAME_NOCONFIG "libhpp-fcl.so"
  )

list(APPEND _cmake_import_check_targets hpp-fcl::hpp-fcl )
list(APPEND _cmake_import_check_files_for_hpp-fcl::hpp-fcl "/home/pth/pth/model/Go2_simulator/devel/lib/libhpp-fcl.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
