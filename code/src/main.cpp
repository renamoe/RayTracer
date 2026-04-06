#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
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
    int width = sceneParser.getCamera()->getWidth();
    int height = sceneParser.getCamera()->getHeight();
    Image outputImage(width, height);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            Ray ray = sceneParser.getCamera()->generateRay(Vector2f(x, y));
            Hit hit;
            bool isHit = sceneParser.getGroup()->intersect(ray, hit, 0);
            if (isHit) {
                Vector3f color = Vector3f::ZERO;
                Vector3f hitPoint = ray.pointAtParameter(hit.getT());
                for (int i = 0; i < sceneParser.getNumLights(); ++i) {
                    Light *light = sceneParser.getLight(i);
                    Vector3f lightDirection, lightColor;
                    light->getIllumination(hitPoint, lightDirection, lightColor);
                    color += hit.getMaterial()->Shade(ray, hit, lightDirection, lightColor);
                }
                outputImage.SetPixel(x, y, color);
            } else {
                outputImage.SetPixel(x, y, sceneParser.getBackgroundColor());
            }
        }
    }
    outputImage.SaveImage(outputFile.c_str());
    
    return 0;
}

