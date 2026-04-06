#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "Vector3f.h"
#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"

#include <string>

using namespace std;

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.

    cout << "Hello! Computer Graphics!" << endl;

    SceneParser sceneParser(inputFile.c_str());
    std::cout << "lights: " << sceneParser.getNumLights() << "\n";
    for (int i = 0; i < sceneParser.getNumLights(); ++i) {
        std::cout << *sceneParser.getLight(i) << "\n";
    }
    int width = sceneParser.getCamera()->getWidth();
    int height = sceneParser.getCamera()->getHeight();
    
    return 0;
}

