#include "model.h"

std::istream& operator>>(std::istream& is, Vertex& v)
{
  char c;
  is >> v.v >> c >> v.vt >> c >> v.vn;
  return is;
}

std::istream& operator>>(std::istream& is, Triangle& t)
{
  is >> t.v1 >> t.v2 >> t.v3;
  return is;
}

void Model::load(std::string filename)
{
  std::ifstream fin(filename);

  std::string key;
  std::string s;
  Vector3f v;
  Triangle t;
  while (fin >> key) {
    switch (key[0]) {
    case 'v':
      fin >> v;
      switch (key[1]) {
      case 't': texts_.push_back(v); break;
      case 'n': norms_.push_back(v); break;
      default: verts_.push_back(v); break;
      };
      break;
    case 'f': fin >> t; faces_.push_back(t); break;
    case 'g': fin >> name_; break;
    case '#': getline(fin, s); break;
    default:  getline(fin, s); break;
    }
  }
}

Vector3f& Model::vert(int i)
{
  return verts_[i - 1];
}

Vector3f& Model::text(int i)
{
  return texts_[i - 1];
}

Vector3f& Model::normal(int i) {
  return norms_[i - 1];
}

Triangle& Model::face(int i)
{
  return faces_[i];
}

std::vector<Triangle>& Model::faces()
{
  return faces_;
}
