#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenKF" for configuration "Debug"
set_property(TARGET OpenKF APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(OpenKF PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libOpenKF.a"
  )

list(APPEND _cmake_import_check_targets OpenKF )
list(APPEND _cmake_import_check_files_for_OpenKF "${_IMPORT_PREFIX}/lib/libOpenKF.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
