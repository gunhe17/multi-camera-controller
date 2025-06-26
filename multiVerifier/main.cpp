#pragma once

#include "class/resource.cpp"
#include "class/camera.cpp"


int main() {
    Resource resource;

    resource.init();
    resource.setup();
    resource.check();
    resource.test();
    resource.cleanup();

    Camera camera;

    camera.init();
    camera.setup();
    camera.check();
    camera.test();
    camera.cleanup();

    return 0;
}