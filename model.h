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

  friend std::istream& operator>>(std::istream&, Triangle&);
};

class Model {
private:
  std::vector<Vector3f> verts_;
  std::vector<Vector3f> texts_;
  std::vector<Vector3f> norms_;
  std::vector<Triangle> faces_;
  std::string name_;

public:
  Model() :name_("") {};

  void load(std::string filename);
  Vector3f& vert(int i);
  Vector3f& text(int i);
  Vector3f& normal(int i);
  Triangle& face(int i);
  std::vector<Triangle>& faces();
  size_t nverts() { return verts_.size(); };
  size_t nfaces() { return faces_.size(); };
};

#endif //__MODEL_H__
