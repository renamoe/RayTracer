#!/usr/bin/env bash

# cmake -B build
# cmake --build build

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output
PROJECT_NAME=RayTracer
build/${PROJECT_NAME} testcases/scene01_basic.txt output/scene01.bmp
build/${PROJECT_NAME} testcases/scene02_cube.txt output/scene02.bmp
build/${PROJECT_NAME} testcases/scene03_sphere.txt output/scene03.bmp
build/${PROJECT_NAME} testcases/scene04_axes.txt output/scene04.bmp
build/${PROJECT_NAME} testcases/scene05_bunny_200.txt output/scene05.bmp
build/${PROJECT_NAME} testcases/scene06_bunny_1k.txt output/scene06.bmp
build/${PROJECT_NAME} testcases/scene07_shine.txt output/scene07.bmp
