#include "ofMain.h"
#include "Plane.h"
#include "Ray.h"


/// \brief it creates a ray given an origin T and a direction T
/// @param [in] _origin
/// @param [in] _direction
ofxraycaster::Ray::Ray(glm::vec3 _origin, glm::vec3 _direction){
    origin = _origin;
    direction = glm::normalize(_direction);
}

/// \brief it sets the origin and direction of a ray. For example,
/// for a 2D ray:
///
/// ~~~~{.cpp}
/// ofxraycaster::Ray<glm::vec2> ray;
/// ray.setup(glm::vec2(10,5), glm::vec2(1,0));
/// ~~~~
void ofxraycaster::Ray::setup(glm::vec3 _origin, glm::vec3 _direction){
    origin = _origin;
    direction = glm::normalize(_direction);
}

/// \brief it returns the origin of the ray,
const glm::vec3 ofxraycaster::Ray::getOrigin() {
    return origin;
}

void ofxraycaster::Ray::setOrigin(glm::vec3 _origin){
    origin = _origin;
}

const glm::vec3 ofxraycaster::Ray::getDirection() {
    return direction;
}

const ofVboMesh & ofxraycaster::Ray::getTrack(){
    return track;
}

void ofxraycaster::Ray::setDirection(glm::vec3 _direction){
    direction = _direction;
}

void ofxraycaster::Ray::setMaxLength(float _maxLength){
    maxLength = _maxLength;
}

void ofxraycaster::Ray::setMaxReflectionNum(unsigned int _maxReflectionNum){
    maxReflectionNum = _maxReflectionNum;
}

void ofxraycaster::Ray::setMinReflectionDistance(float _minReflectionDistance){
    minReflectionDistance = _minReflectionDistance;
}

void ofxraycaster::Ray::draw(float radius){
    ofPushStyle();
    // draw origin
    ofSetColor(246,233,101);
    ofDrawSphere(origin, radius);

    // draw direction
    auto end = origin + (direction * (radius*10.0f));
    ofSetLineWidth(3);
    ofDrawLine(origin,end);
    ofSetLineWidth(1);
    ofPopStyle();
}

bool ofxraycaster::Ray::intersectsPlane(ofxraycaster::Plane plane, float & distance){
    return glm::intersectRayPlane(origin, direction,
                                  plane.getOrigin(), plane.getNormal(),
                                  distance);
}

bool ofxraycaster::Ray::intersectsTriangle(glm::vec3 const & vert0, glm::vec3 const & vert1, glm::vec3 const & vert2, glm::vec3 & baryPosition){

    return glm::intersectRayTriangle(origin, direction, vert0, vert1, vert2, baryPosition);
}

bool ofxraycaster::Ray::intersectsSphere(const glm::vec3 & _center, const float & _radius, glm::vec3& _position, glm::vec3 & _normal){

    return glm::intersectRaySphere(origin, direction, _center, _radius, _position, _normal);
}

bool ofxraycaster::Ray::intersectsPrimitiveMultiCast(const of3dPrimitive& primitive){

    track.setMode(OF_PRIMITIVE_LINE_STRIP);
    track.clear();
    
    glm::vec3 ori = origin;
    glm::vec3 dir = direction;
    
    float rayLength = 0;
    
    vector<glm::vec3> & vs = track.getVertices();
    vs.emplace_back(ori);
    
    while(1){
        
//        cout << endl;
//        cout << "Ray       " << vs.size() << endl;
//        cout << "origin    " << ori << endl;
//        cout << "direction " << dir << endl;
        
        // at the beginning, no intersection is found and the distance to the closest surface
        // is set to an high value;
        bool found = false;
        glm::vec3 baricentricCoords;
        glm::vec3 intNormal;
        float distanceToTheClosestSurface = numeric_limits<float>::max();
        for (const ofMeshFace& face : primitive.getMesh().getUniqueFaces()) {
            bool intersection = glm::intersectRayTriangle(
                                                          ori, dir,
                                                          glm::vec3(primitive.getGlobalTransformMatrix() * glm::vec4(face.getVertex(0), 1.f)),
                                                          glm::vec3(primitive.getGlobalTransformMatrix() * glm::vec4(face.getVertex(1), 1.f)),
                                                          glm::vec3(primitive.getGlobalTransformMatrix() * glm::vec4(face.getVertex(2), 1.f)),
                                                          baricentricCoords);
            // when an intersection is found, it updates the distanceToTheClosestSurface value
            // this value is used to order the new intersections, if a new intersection with a smaller baricenter.z
            // value is found, this one will become the new intersection
            if (intersection) {
                if (baricentricCoords.z < distanceToTheClosestSurface) {
                    
                    // check if resulted position is not the same one
                    if(baricentricCoords.z > minReflectionDistance){
                        found = true;
                        distanceToTheClosestSurface = baricentricCoords.z;
                        
                        intNormal = glm::normalize(
                                                   glm::vec3(primitive.getGlobalTransformMatrix() *
                                                             glm::vec4(face.getFaceNormal(), 1.0f))
                                                   );
                    }
                }
            }
        }
        baricentricCoords.z = distanceToTheClosestSurface;
        
        if(found){
            float d = baricentricCoords.z;  // distance from origin <-> hit point
            
            // 1. check ray distance
            if(maxLength <= rayLength + d){
                d = maxLength - rayLength;
                glm::vec3 hitPos = ori + dir * d;
                vs.emplace_back( hitPos );
                // layLength = maxLength
                break;
            }else{
                glm::vec3 hitPos = ori + dir * d;
                vs.emplace_back( hitPos );
                rayLength += d;
                
                // 2. check how many times does this ray be reflected
                if(vs.size() > maxReflectionNum){
                    break;
                }else{
                    // we keep casting ray, so update origin and direction
                    ori = hitPos;
                    // reflect
                    glm::vec3 ref = glm::reflect(dir, intNormal);
                    dir = normalize(ref);
                }
            }
        }else{
            
            if(vs.size() == 1){
                // nothing to hit
                break;
            }else{
                // hit several times already
                // lets extend a ray until max length
                float d = maxLength - rayLength;
                glm::vec3 endPos = ori + dir * d;
                vs.emplace_back( endPos );
                break;
            }
        }
    }
    
    return (vs.size() > 1);
}

bool ofxraycaster::Ray::intersectsPrimitive(const of3dPrimitive& primitive,  glm::vec3 & baricentricCoords, glm::vec3 & intNormal) {
    // at the beginning, no intersection is found and the distance to the closest surface
    // is set to an high value;
    bool found = false;
    float distanceToTheClosestSurface = numeric_limits<float>::max();
    for (const ofMeshFace& face : primitive.getMesh().getUniqueFaces()) {
        bool intersection = glm::intersectRayTriangle(
                                                      origin, direction,
                                                      glm::vec3(primitive.getGlobalTransformMatrix() * glm::vec4(face.getVertex(0), 1.f)),
                                                      glm::vec3(primitive.getGlobalTransformMatrix() * glm::vec4(face.getVertex(1), 1.f)),
                                                      glm::vec3(primitive.getGlobalTransformMatrix() * glm::vec4(face.getVertex(2), 1.f)),
                                                      baricentricCoords);
        // when an intersection is found, it updates the distanceToTheClosestSurface value
        // this value is used to order the new intersections, if a new intersection with a smaller baricenter.z
        // value is found, this one will become the new intersection
        if (intersection) {
            if (baricentricCoords.z < distanceToTheClosestSurface) {
                found = true;
                distanceToTheClosestSurface = baricentricCoords.z;

                intNormal = glm::normalize(
                   glm::vec3(primitive.getGlobalTransformMatrix() *
                             glm::vec4(face.getFaceNormal(), 1.0f))
                );
            }
        }
    }
    baricentricCoords.z = distanceToTheClosestSurface;
    return found;
}

bool ofxraycaster::Ray::intersectsMesh(const ofMesh& mesh,  glm::vec3 & baricentricCoords, glm::vec3 & intNormal){

    // Guards. intersectsMesh only works with indexed geometries of
    // traingles
    if (mesh.getMode() != OF_PRIMITIVE_TRIANGLES) {
        ofLog() << "intersectsMesh works only with OF_PRIMITIVE_TRIANGLES";
        return false;
    }

    if (mesh.getNumIndices() < 3) {
        ofLog() << "mesh intersection works only with indexed geometries";
        return false;
    }

    if (mesh.getNumIndices()%3 != 0) {
        ofLog() << "the total number of the indices is not a multiple of 3";
        return false;
    }

    // at the beginning, no intersection is found and the distance
    // to the closest surface is set to an high value;
    bool found = false;
    float distanceToTheClosestSurface = numeric_limits<float>::max();
    for (unsigned int i = 0; i< mesh.getNumIndices(); i+=3) {
        bool intersection = glm::intersectRayTriangle(
                              origin, direction,
                              mesh.getVertex(mesh.getIndex(i)),
                              mesh.getVertex(mesh.getIndex(i+1)),
                              mesh.getVertex(mesh.getIndex(i+2)),
                              baricentricCoords
        );

        // when an intersection is found, it updates the distanceToTheClosestSurface value
        // this value is used to order the new intersections, if a new intersection with a smaller baricenter.z
        // value is found, this one will become the new intersection
        if (intersection) {
            if (baricentricCoords.z < distanceToTheClosestSurface) {
                found = true;
                distanceToTheClosestSurface = baricentricCoords.z;

                const int ia = mesh.getIndex(i);
                const int ib = mesh.getIndex(i+1);
                const int ic = mesh.getIndex(i+2);

                glm::vec3 e1 = mesh.getVertex(ia) - mesh.getVertex(ib);
                glm::vec3 e2 = mesh.getVertex(ic) - mesh.getVertex(ib);

                intNormal = glm::cross(e1,e2);
            }
        }
    }
    baricentricCoords.z = distanceToTheClosestSurface;
    return found;
}

bool ofxraycaster::Ray::intersectsMesh(const ofMesh& mesh, const glm::mat4& transformationMatrix, glm::vec3 & baricentricCoords, glm::vec3 & intNormal){

    // Guards. intersectsMesh only works with indexed geometries of
    // traingles
    if (mesh.getMode() != OF_PRIMITIVE_TRIANGLES) {
        ofLog() << "intersectsMesh works only with OF_PRIMITIVE_TRIANGLES";
        return false;
    }

    if (mesh.getNumIndices() < 3) {
        ofLog() << "mesh intersection works only with indexed geometries";
        return false;
    }

    if (mesh.getNumIndices()%3 != 0) {
        ofLog() << "the total number of the indices is not a multiple of 3";
        return false;
    }

    // at the beginning, no intersection is found and the distance
    // to the closest surface is set to an high value;
    bool found = false;
    float distanceToTheClosestSurface = numeric_limits<float>::max();
    for (unsigned int i = 0; i< mesh.getNumIndices(); i+=3) {
        bool intersection = glm::intersectRayTriangle(
                              origin, direction,
                              glm::vec3(transformationMatrix * glm::vec4(mesh.getVertex(mesh.getIndex(i)), 1.0f)),
                              glm::vec3(transformationMatrix * glm::vec4(mesh.getVertex(mesh.getIndex(i+1)), 1.0f)),
                              glm::vec3(transformationMatrix * glm::vec4(mesh.getVertex(mesh.getIndex(i+2)), 1.0f)),
                              baricentricCoords
        );


        if (intersection) {
            if (baricentricCoords.z < distanceToTheClosestSurface) {
                found = true;
                distanceToTheClosestSurface = baricentricCoords.z;

                const int ia = mesh.getIndex(i);
                const int ib = mesh.getIndex(i+1);
                const int ic = mesh.getIndex(i+2);

                glm::vec3 e1 = mesh.getVertex(ia) - mesh.getVertex(ib);
                glm::vec3 e2 = mesh.getVertex(ic) - mesh.getVertex(ib);
                glm::vec4 no = glm::vec4(glm::cross(e1,e2), 1.0f);
                
                intNormal = glm::vec3(transformationMatrix * no);
            }
        }
    }
    baricentricCoords.z = distanceToTheClosestSurface;
    return found;
}
