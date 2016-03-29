#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include "tgaimage.h"
#include "model.h"
#include "texture.h"

const int width = 800;
const int height = 800;
const int depth = 255;

const Vector3f kLightDirection{ 1.f / 3, 1.f / 3, 1.f / 3 };
//const Vector3f kLightDirection{ 0, 0, 1 };
const Vector3f            kEye{ 1, 1, 3 };
const Vector3f         kCenter{ 0, 0, 0 };
const Vector3f             kUp{ 0, 1, 0 };

const TGAColor red = TGAColor(depth, 0, 0, depth);
const TGAColor white = TGAColor(depth, depth, depth, depth);

class IShader {
public:
  virtual Matrix43f Vertex(int face_idx) = 0;
  virtual bool Fragment(Vector3f barycentric_coords, TGAColor& color) = 0;
};

class Shader : public IShader {
public:
  Shader(Model& model, Texture& texture, Texture& normal_map, Matrix44f uniform_world, Matrix44f uniform_M) 
    : model_(model), uniform_world_(uniform_world), texture_(texture), normal_map_(normal_map), 
    uniform_M_(uniform_M), uniform_MIT_(uniform_M_.InvertTranspose())
  {}

  virtual Matrix43f Vertex(int face_idx) {
    Matrix43f world_coordinates = model_.GetFaceVertexCoordinates(face_idx);
    varying_texture_coordinates_ = model_.GetFaceTextureCoordinates(face_idx);
    varying_face_normals_ = model_.GetFaceNormals(face_idx);
    
    return uniform_world_ * world_coordinates;
  }

  virtual bool Fragment(Vector3f barycentric_coords, TGAColor& color) {
    auto texture_uv_coords = varying_texture_coordinates_ * barycentric_coords;

    TGAColor normal_color = normal_map_.Get(texture_uv_coords.x, texture_uv_coords.y);
    Vector3f normal{ normal_color[2] / 255.f * 2.f - 1.f, normal_color[1] / 255.f * 2.f - 1.f, normal_color[0] / 255.f * 2.f - 1.f };

    Vector3f n = FromHomogeneous((uniform_MIT_ * ToHomogeneous(normal)).Normalize());
    Vector3f l = FromHomogeneous((uniform_M_ * ToHomogeneous(kLightDirection)).Normalize());
    float intensity = std::max(0.f, (n * l));

    color = texture_.Get(texture_uv_coords.x, texture_uv_coords.y);
    color.r *= intensity;
    color.g *= intensity;
    color.b *= intensity;

    return false;
  }

private:
  const Model& model_;
  const Texture& texture_;
  const Texture& normal_map_;

  Matrix44f uniform_world_;
  Matrix44f uniform_M_;
  Matrix44f uniform_MIT_;

  Matrix43f varying_clip_coordinates_;
  Matrix23f varying_texture_coordinates_;
  Matrix33f varying_face_normals_;
};

class Screen {
public:
  Screen(int width, int height)
    : width_(width), height_(height), zbuffer_(height_ * width_, -std::numeric_limits<float>::max()),
    screen_buffer_(width_, height_, TGAImage::RGB)
  {}

  bool Set(int x, int y, const TGAColor &c) {
    return screen_buffer_.set(x, y, c);
  }

  bool Set(int x, int y, float z, const TGAColor &c) {
    if (z > zbuffer_[x + y * width_]) {
      zbuffer_[x + y * width_] = z;
      return this->Set(x, y, c);
    }
    else {
      return false;
    }
  }

  void Line(int x0, int y0, int x1, int y1, const TGAColor &color)
  {
    bool transpose = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
      std::swap(x0, y0);
      std::swap(x1, y1);
      transpose = true;
    }

    if (x1 < x0) {
      std::swap(x0, x1);
      std::swap(y0, y1);
    }

    int dy = y1 - y0;

    float r = 1.f / (x1 - x0);
    for (int x = x0; x <= x1; ++x) {
      int y = (x - x0) * r * dy + y0;
      if (transpose) {
        this->Set(y, x, color);
      }
      else {
        this->Set(x, y, color);
      }
    }
  }

  void Triangle(Matrix43f float_clip_coords, IShader& shader) {
    Matrix43i clip_coords = Round(float_clip_coords);

    Vector2i min_bound = { Min(clip_coords[0]), Min(clip_coords[1]) };
    Vector2i max_bound = { Max(clip_coords[0]), Max(clip_coords[1]) };

    min_bound.x = std::max(min_bound.x, 0);
    min_bound.y = std::max(min_bound.y, 0);

    max_bound.x = std::min(max_bound.x, this->get_width() - 1);
    max_bound.y = std::min(max_bound.y, this->get_height() - 1);

    Vector2i a = { clip_coords[0][0], clip_coords[1][0] };
    Vector2i b = { clip_coords[0][1], clip_coords[1][1] };
    Vector2i c = { clip_coords[0][2], clip_coords[1][2] };

    TGAColor color;
    for (int x = min_bound.x; x <= max_bound.x; ++x) {
      for (int y = min_bound.y; y <= max_bound.y; ++y) {
        Vector3f bc = Barycentric(a, b, c, Vector2i{ x, y });
        if (bc.x < 0 || bc.y < 0 || bc.z < 0) {
          continue;
        }

        bool discard = shader.Fragment(bc, color);
        if (!discard) {
          float z_depth = bc * float_clip_coords[2];
          this->Set(x, y, z_depth, color);
        }
      }
    }
  }

  bool WriteTgaFile(const std::string& filename) {
    screen_buffer_.flip_vertically();
    bool result = screen_buffer_.write_tga_file("output.tga");
    screen_buffer_.flip_vertically();
    return result;
  }

  int get_width() const {
    return width_;
  }

  int get_height() const {
    return height_;
  }

public:
  int width_;
  int height_;

  TGAImage screen_buffer_;
  std::vector<float> zbuffer_;
};

Matrix44f SetViewPort(int x, int y, int w, int h, int zbuffer_depth=255) {
  Matrix44f viewport_ = Matrix44f::Identity();

  viewport_[0][3] = x + w / 2.f;
  viewport_[1][3] = y + h / 2.f;
  viewport_[2][3] = zbuffer_depth / 2.f;

  viewport_[0][0] = w / 2.f;
  viewport_[1][1] = h / 2.f;
  viewport_[2][2] = zbuffer_depth / 2.f;

  return viewport_;
}

Matrix44f LookAt(Vector3f eye, Vector3f center, Vector3f up) {
  Matrix44f modelview;
  Vector3f z = (eye - center).Normalize();
  Vector3f x = CrossProduction(up, z).Normalize();
  Vector3f y = CrossProduction(z, x).Normalize();
  Matrix44f Minv = Matrix44f::Identity();
  Matrix44f Tr = Matrix44f::Identity();
  for (int i = 0; i < 3; i++) {
    Minv[0][i] = x[i];
    Minv[1][i] = y[i];
    Minv[2][i] = z[i];
    Tr[i][3] = -center[i];
  }
  modelview = Minv*Tr;
  return modelview;
}

int main(int argc, char** argv)
{
  Screen screen(width, height);
 
  Model model;
  model.load("obj/african_head.obj");
  // model.load("obj/reconstructed_head.obj");
  
  std::cout << "Model loaded" << std::endl;
  std::cout << "nverts: " << model.nverts() << std::endl;
  std::cout << "nfaces: " << model.nfaces() << std::endl;

  Texture texture;
  texture.Load("obj/african_head_diffuse.tga");

  Texture normal_map;
  normal_map.Load("obj/african_head_nm.tga");


  Matrix44f projection = Matrix44f::Identity();
  Matrix44f viewport = SetViewPort(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
  Matrix44f modelview = LookAt(kEye, kCenter, kUp);

  TGAImage image(width, height, TGAImage::RGB);

  Shader shader(model, texture, normal_map, viewport * projection * modelview, projection * modelview);
  
  for (size_t f = 0; f < model.num_faces(); ++f) {
    auto clip_coords = shader.Vertex(f);
    screen.Triangle(clip_coords, shader);
  }

  screen.WriteTgaFile("output.tga");
  return 0;
}

