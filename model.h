#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include <tuple>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

#include "geometry.h"

class Vertex {
public:
  int v;
  int vt;
  int vn;

  friend std::istream& operator>>(std::istream&, Vertex&);
};

class Triangle {
public:
  Vertex v1;
  Vertex v2;
  Vertex v3;

  Vertex& operator[](size_t idx) {
    switch (idx) {
    case 0: return v1;
    case 1: return v2;
    case 2: return v3;
    }
  }

  const Vertex& operator[](size_t idx) const {
    switch (idx) {
    case 0: return v1;
    case 1: return v2;
    case 2: return v3;
    }
  }


  friend std::istream& operator>>(std::istream&, Triangle&);
};

class Model {
public:
  Model() :name_("") {};

  void load(std::string filename);

  Vector3f& GetVertex(int i);
  const Vector3f& GetVertex(int i) const;

  Vector2f& GetTexture(int i);
  const Vector2f& GetTexture(int i) const;

  Vector3f& GetNormal(int i);
  const Vector3f& GetNormal(int i) const;
  
  Triangle& GetFace(int i);
  const Triangle& GetFace(int i) const;

  std::vector<Triangle>& faces();
  size_t nverts() { return vertices_.size(); };
  size_t nfaces() { return faces_.size(); };
  size_t num_faces() { return faces_.size(); };

  Matrix43f GetFaceVertexCoordinates(int i) const;
  Matrix23f GetFaceTextureCoordinates(int i) const;
  Matrix33f GetFaceNormals(int i) const;

private:
  std::vector<Vector3f> vertices_;
  std::vector<Vector2f> textures_;
  std::vector<Vector3f> normals_;
  std::vector<Triangle> faces_;
  std::string name_;
};

#endif //__MODEL_H__
