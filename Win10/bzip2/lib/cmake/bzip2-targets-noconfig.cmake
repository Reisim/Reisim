#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "BZip2::BZip2::BZip2" for configuration ""
set_property(TARGET BZip2::BZip2::BZip2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(BZip2::BZip2::BZip2 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libbz2.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS BZip2::BZip2::BZip2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_BZip2::BZip2::BZip2 "${_IMPORT_PREFIX}/lib/libbz2.a" )

# Import target "BZip2::bzip2" for configuration ""
set_property(TARGET BZip2::bzip2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(BZip2::bzip2 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/bzip2.exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS BZip2::bzip2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_BZip2::bzip2 "${_IMPORT_PREFIX}/bin/bzip2.exe" )

# Import target "BZip2::bzip2recover" for configuration ""
set_property(TARGET BZip2::bzip2recover APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(BZip2::bzip2recover PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/bzip2recover.exe"
  )

list(APPEND _IMPORT_CHECK_TARGETS BZip2::bzip2recover )
list(APPEND _IMPORT_CHECK_FILES_FOR_BZip2::bzip2recover "${_IMPORT_PREFIX}/bin/bzip2recover.exe" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
