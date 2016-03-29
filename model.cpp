#include "model.h"

std::istream& operator>>(std::istream& is, Vertex& v) {
  std::string s; is >> s;
  v.v = v.vt = v.vn = 0;
  int pos = 0;
  while (s[pos] != '/') {
    v.v = v.v * 10 + (s[pos] - '0');
    ++pos;
  }
  ++pos;

  while (s[pos] != '/') {
    v.vt = v.vt * 10 + (s[pos] - '0');
    ++pos;
  }
  ++pos;
  while (pos < s.size()) {
    v.vn = v.vn * 10 + (s[pos] - '0');
    ++pos;
  }

  return is;
}

std::istream& operator>>(std::istream& is, Triangle& t) {
  is >> t.v1 >> t.v2 >> t.v3;
  return is;
}

void Model::load(std::string filename) {
  std::ifstream fin(filename);

  std::string key;
  std::string s;
  Vector3f v;
  Vector2f v2;
  Triangle t;
  
  while (fin >> key) {
    if (key == "v") {
      fin >> v;
      vertices_.push_back(v);
    } else if (key == "vt") {
      fin >> v2;
      textures_.push_back(v2);
    } else if (key == "vn") {
      fin >> v;
      normals_.push_back(v);
    } else if (key == "f") {
      fin >> t;
      faces_.push_back(t);
    } else if (key == "g") {
      fin >> name_;
    } else {
      getline(fin, s);
    }
  }
}

Vector3f& Model::GetVertex(int i) {
  return vertices_[i - 1];
}

const Vector3f& Model::GetVertex(int i) const {
  return vertices_[i - 1];
}

Vector2f& Model::GetTexture(int i) {
  return textures_[i - 1];
}

const Vector2f& Model::GetTexture(int i) const {
  return textures_[i - 1];
}

Vector3f& Model::GetNormal(int i) {
  return normals_[i - 1];
}

const Vector3f& Model::GetNormal(int i) const {
  return normals_[i - 1];
}

Triangle& Model::GetFace(int i) {
  return faces_[i];
}

const Triangle& Model::GetFace(int i) const {
  return faces_[i];
}

std::vector<Triangle>& Model::faces() {
  return faces_;
}

Matrix43f Model::GetFaceVertexCoordinates(int i) const {
  const Triangle& t = faces_[i];
  const Vector3f& world_v1 = GetVertex(t.v1.v);
  const Vector3f& world_v2 = GetVertex(t.v2.v);
  const Vector3f& world_v3 = GetVertex(t.v3.v);

  Matrix43f world_coordinates;
  world_coordinates.SetCol(0, ToHomogeneous(world_v1));
  world_coordinates.SetCol(1, ToHomogeneous(world_v2));
  world_coordinates.SetCol(2, ToHomogeneous(world_v3));

  return world_coordinates;
}

Matrix23f Model::GetFaceTextureCoordinates(int i) const {
  const Triangle& t = faces_[i];

  const Vector2f& texture_v1 = GetTexture(t.v1.vt);
  const Vector2f& texture_v2 = GetTexture(t.v2.vt);
  const Vector2f& texture_v3 = GetTexture(t.v3.vt);

  Matrix23f triangle_texture_coordinates;
  triangle_texture_coordinates.SetCol(0, texture_v1);
  triangle_texture_coordinates.SetCol(1, texture_v2);
  triangle_texture_coordinates.SetCol(2, texture_v3);

  return triangle_texture_coordinates;
}

Matrix33f Model::GetFaceNormals(int i) const {
  const Triangle& t = faces_[i];

  const Vector3f& normal_v1 = GetNormal(t.v1.vn);
  const Vector3f& normal_v2 = GetNormal(t.v2.vn);
  const Vector3f& normal_v3 = GetNormal(t.v3.vn);

  Matrix33f triangle_normal;
  triangle_normal.SetCol(0, normal_v1);
  triangle_normal.SetCol(1, normal_v2);
  triangle_normal.SetCol(2, normal_v3);

  return triangle_normal;
}