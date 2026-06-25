CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(RayTracer CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

ADD_SUBDIRECTORY(deps/vecmath)

SET(RayTracer_SOURCES
        src/aabb.cpp
        src/bdpt.cpp
        src/bsdf.cpp
        src/camera.cpp
        src/group.cpp
        src/hit.cpp
        src/image.cpp
        src/light.cpp
        src/material.cpp
        src/main.cpp
        src/mesh.cpp
        src/path_guiding.cpp
        src/path_tracer.cpp
        src/preview_window.cpp
        src/plane.cpp
        src/progress.cpp
        src/ray.cpp
        src/render_config.cpp
        src/renderer.cpp
        src/scene_parser.cpp
        src/sphere.cpp
        src/texture.cpp
        src/tone_mapping.cpp
        src/transform.cpp
        src/triangle.cpp
        src/vcm.cpp
        src/hash_grid.cpp)

SET(RayTracer_INCLUDES
        include/aabb.hpp
        include/bdpt.hpp
        include/bsdf.hpp
        include/camera.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/light.hpp
        include/material.hpp
        include/mesh.hpp
        include/object3d.hpp
        include/path_guiding.hpp
        include/path_tracer.hpp
        include/preview_window.hpp
        include/plane.hpp
        include/progress.hpp
        include/ray.hpp
        include/render_config.hpp
        include/renderer.hpp
        include/scene_parser.hpp
        include/sphere.hpp
        include/texture.hpp
        include/tone_mapping.hpp
        include/transform.hpp
        include/triangle.hpp
        include/random.hpp
        include/vcm.hpp
        include/hash_grid.hpp
        include/path_vertex.hpp)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

ADD_EXECUTABLE(${PROJECT_NAME} ${RayTracer_SOURCES} ${RayTracer_INCLUDES})
find_package(OpenMP REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX vecmath)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)

option(RAYTRACER_ENABLE_PREVIEW "Enable SDL2 realtime preview window" ON)
if(RAYTRACER_ENABLE_PREVIEW)
    find_package(SDL2 QUIET)
    if(SDL2_FOUND)
        target_compile_definitions(${PROJECT_NAME} PRIVATE RAYTRACER_ENABLE_PREVIEW)
        if(TARGET SDL2::SDL2)
            target_link_libraries(${PROJECT_NAME} PRIVATE SDL2::SDL2)
        else()
            target_include_directories(${PROJECT_NAME} PRIVATE ${SDL2_INCLUDE_DIRS})
            target_link_libraries(${PROJECT_NAME} PRIVATE ${SDL2_LIBRARIES})
        endif()
    else()
        find_package(PkgConfig QUIET)
        if(PKG_CONFIG_FOUND)
            pkg_check_modules(SDL2_PKG QUIET sdl2)
        endif()

        if(SDL2_PKG_FOUND)
            target_compile_definitions(${PROJECT_NAME} PRIVATE RAYTRACER_ENABLE_PREVIEW)
            target_include_directories(${PROJECT_NAME} PRIVATE ${SDL2_PKG_INCLUDE_DIRS})
            target_compile_options(${PROJECT_NAME} PRIVATE ${SDL2_PKG_CFLAGS_OTHER})
            target_link_libraries(${PROJECT_NAME} PRIVATE ${SDL2_PKG_LDFLAGS})
        else()
            message(WARNING "SDL2 not found; realtime preview support will be disabled.")
        endif()
    endif()
endif()
