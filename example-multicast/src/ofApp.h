#pragma once

#include "ofMain.h"
#include "ofxRaycaster.h"

class ofApp : public ofBaseApp{
public:
    void setup();
    void update();
    void draw();
    void keyPressed(int key);
    void drawPrimitiveIntersection();

    ofEasyCam camera;
    std::string mode = "Primitive";
    glm::vec3 lookAtRay = glm::vec3(0, 200, 0);

    ofBoxPrimitive box;

    glm::vec3 rayDirection;
    glm::vec3 rayOrigin;
    ofxraycaster::Ray ray;

};
