find_path(OpenGLRecorder_INCLUDE_DIR NAMES openglrecorder.h)
find_library(OpenGLRecorder_LIBRARY openglrecorder)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenGLRecorder DEFAULT_MSG OpenGLRecorder_LIBRARY OpenGLRecorder_INCLUDE_DIR)

mark_as_advanced(OpenGLRecorder_LIBRARY OpenGLRecorder_INCLUDE_DIR)

if(NOT TARGET OpenGLRecorder)
    add_library(OpenGLRecorder UNKNOWN IMPORTED)
    set_property(TARGET OpenGLRecorder PROPERTY IMPORTED_LOCATION ${OpenGLRecorder_LIBRARY})
    set_property(TARGET OpenGLRecorder PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OpenGLRecorder_INCLUDE_DIR})
    list(APPEND OpenGLRecorder_TARGETS OpenGLRecorder)
endif()