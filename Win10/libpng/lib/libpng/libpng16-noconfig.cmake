#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "png" for configuration ""
set_property(TARGET png APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(png PROPERTIES
  IMPORTED_IMPLIB_NOCONFIG "${_IMPORT_PREFIX}/lib/libpng16.dll.a"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/libpng16.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS png )
list(APPEND _IMPORT_CHECK_FILES_FOR_png "${_IMPORT_PREFIX}/lib/libpng16.dll.a" "${_IMPORT_PREFIX}/bin/libpng16.dll" )

# Import target "png_static" for configuration ""
set_property(TARGET png_static APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(png_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libpng16.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS png_static )
list(APPEND _IMPORT_CHECK_FILES_FOR_png_static "${_IMPORT_PREFIX}/lib/libpng16.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
