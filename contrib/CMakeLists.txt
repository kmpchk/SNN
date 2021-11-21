if (NOT EXISTS "${PROJECT_SOURCE_DIR}/contrib/spdlog/CMakeLists.txt")
    message(FATAL_ERROR "submodule contrib/spdlog is missing.
         To fix try run: \n git submodule update --init --recursive")
else ()
    add_subdirectory(${PROJECT_SOURCE_DIR}/contrib/spdlog ${CMAKE_CURRENT_BINARY_DIR}/spdlog)
endif ()

if (NOT EXISTS "${PROJECT_SOURCE_DIR}/contrib/h5pp/CMakeLists.txt")
    message(FATAL_ERROR "submodule contrib/h5pp is missing.
         To fix try run: \n git submodule update --init --recursive")
else ()
    add_subdirectory(${PROJECT_SOURCE_DIR}/contrib/h5pp ${CMAKE_CURRENT_BINARY_DIR}/h5pp)
    # Speed up compilation with precompiled headers
    if (H5PP_ENABLE_PCH)
        message("H5PP_ENABLE_PCH")
    endif()
endif ()