#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "blueboat_interfaces::blueboat_interfaces__rosidl_generator_py" for configuration "RelWithDebInfo"
set_property(TARGET blueboat_interfaces::blueboat_interfaces__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(blueboat_interfaces::blueboat_interfaces__rosidl_generator_py PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libblueboat_interfaces__rosidl_generator_py.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libblueboat_interfaces__rosidl_generator_py.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS blueboat_interfaces::blueboat_interfaces__rosidl_generator_py )
list(APPEND _IMPORT_CHECK_FILES_FOR_blueboat_interfaces::blueboat_interfaces__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libblueboat_interfaces__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
