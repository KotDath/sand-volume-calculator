#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Open3D::Open3D" for configuration "Debug"
set_property(TARGET Open3D::Open3D APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(Open3D::Open3D PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/Open3D.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/Open3D.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS Open3D::Open3D )
list(APPEND _IMPORT_CHECK_FILES_FOR_Open3D::Open3D "${_IMPORT_PREFIX}/lib/Open3D.lib" "${_IMPORT_PREFIX}/bin/Open3D.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
