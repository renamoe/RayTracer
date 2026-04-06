CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(RayTracer CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

ADD_SUBDIRECTORY(deps/vecmath)

SET(RayTracer_SOURCES
        src/camera.cpp
        src/group.cpp
        src/hit.cpp
        src/image.cpp
        src/light.cpp
        src/material.cpp
        src/main.cpp
        src/mesh.cpp
        src/plane.cpp
        src/ray.cpp
        src/scene_parser.cpp
        src/sphere.cpp
        src/transform.cpp
        src/triangle.cpp)

SET(RayTracer_INCLUDES
        include/camera.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/light.hpp
        include/material.hpp
        include/mesh.hpp
        include/object3d.hpp
        include/plane.hpp
        include/ray.hpp
        include/scene_parser.hpp
        include/sphere.hpp
        include/transform.hpp
        include/triangle.hpp
        include/random.hpp
        )

SET(CMAKE_CXX_STANDARD 23)

ADD_EXECUTABLE(${PROJECT_NAME} ${RayTracer_SOURCES} ${RayTracer_INCLUDES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} vecmath)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)
