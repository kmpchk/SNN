if (NOT EXISTS "${PROJECT_SOURCE_DIR}/contrib/spdlog/CMakeLists.txt")
    message(FATAL_ERROR "submodule contrib/spdlog is missing.
         To fix try run: \n git submodule update --init --recursive")
else ()
    add_subdirectory(${PROJECT_SOURCE_DIR}/contrib/spdlog ${CMAKE_CURRENT_BINARY_DIR}/spdlog)
endif ()

if (NOT EXISTS "${PROJECT_SOURCE_DIR}/contrib/csv-parser/CMakeLists.txt")
    message(FATAL_ERROR "submodule contrib/csv-parser is missing.
         To fix try run: \n git submodule update --init --recursive")
else ()
    add_subdirectory(${PROJECT_SOURCE_DIR}/contrib/csv-parser ${CMAKE_CURRENT_BINARY_DIR}/csv-parser)
    # fix headers missing problem
    set(DEPENDENCIES_DIR ${PROJECT_SOURCE_DIR}/contrib)
    # a workaround to expose CSV_INCLUDE_DIR for root CMakeListst.txt
    set(CSV_INCLUDE_DIR ${DEPENDENCIES_DIR}/csv-parser/include PARENT_SCOPE)
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