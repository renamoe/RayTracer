#include <iostream>

#include "render_config.hpp"
#include "renderer.hpp"
#include "scene_parser.hpp"

int main(int argc, char *argv[]) {
    RenderConfig config;
    if (!parseRenderConfig(argc, argv, config)) {
        return 1;
    }

    std::cout << "Hello! Computer Graphics!" << std::endl;

    SceneParser scene(config.inputFile.c_str());
    std::cout << "num lights: " << scene.getNumLights() << "\n";

    Renderer renderer(scene, config);
    Image image = renderer.render();
    image.SaveBMP(config.outputFile.c_str());

    return 0;
}
