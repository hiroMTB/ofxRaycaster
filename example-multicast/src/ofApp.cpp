#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(100);

    //box
    box.set(300);
    box.setPosition(0,0,0);
    
    // ray
    ray.setOrigin( {0,10,0});
    ray.setDirection({0.2,1,0.4});
    ray.setMaxLength(10000);
    ray.setMaxReflectionNum(100);
    
}

//--------------------------------------------------------------
void ofApp::update(){

    float angle = fmodf(ofGetFrameNum()*0.1, 360);
    box.setOrientation( glm::vec3(angle) );
    ray.intersectsPrimitiveMultiCast(box);
}

//--------------------------------------------------------------
void ofApp::draw(){

    ofEnableDepthTest();
    camera.begin();
    
    ofSetColor(255,0,0);
    ray.getTrack().draw();

    ofSetColor(255);
    box.drawWireframe();

    camera.end();
};


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

