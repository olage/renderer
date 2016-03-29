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

//const Vector3f kLightDirection{ 1.f / 3, 1.f / 3, 1.f / 3 };
//const Vector3f kLightDirection{ 1.f/5, 1.f/5, 3.f/5 };
const Vector3f kLightDirection{ 0, 0, 1 };
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
  Matrix43f varying_clip_coordinates;
  Matrix23f varying_texture_coordinates;
  Matrix33f varying_face_normals;

  Shader(Model& model, Texture& texture, Matrix44f uniform_world) 
    : model_(model), uniform_world_(uniform_world), texture_(texture)
  {}

  virtual Matrix43f Vertex(int face_idx) {
    Matrix43f world_coordinates = model_.GetFaceVertexCoordinates(face_idx);
    varying_texture_coordinates = model_.GetFaceTextureCoordinates(face_idx);
    varying_face_normals = model_.GetFaceNormals(face_idx);
    
    varying_clip_coordinates = uniform_world_ * world_coordinates;
    return varying_clip_coordinates;
  }

  virtual bool Fragment(Vector3f barycentric_coords, TGAColor& color) {
    auto normal = varying_face_normals * barycentric_coords;
    float intensity = DotProduction(normal, kLightDirection);
    if (intensity < 0) intensity = 0;

    auto clip_coords = varying_clip_coordinates * barycentric_coords;

    auto texture_coords = varying_texture_coordinates * barycentric_coords;
    color = texture_.Get(texture_coords.x, texture_coords.y);
    color.r *= intensity;
    color.g *= intensity;
    color.b *= intensity;

    return false;
  }

private:
  const Model& model_;
  const Texture& texture_;
  Matrix44f uniform_world_;
};


class Screen {
public:
  Screen(int width, int height)
    : width_(width), height_(height), zbuffer_(height_ * width_, -std::numeric_limits<float>::max()),
    screen_buffer_(width_, height_, TGAImage::RGB), projection_(Matrix44f::Identity())
  {
    SetViewPort(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
 //   SetViewPort(width / 2 - 20, height / 2 - 20, 40, 40);
    LookAt(kEye, kCenter, kUp);
  }

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

  void Triangle(Matrix43f world_coords, const TGAColor& color) {
    Matrix43f float_clip_coords = viewport_ * projection_ * modelview_ * (world_coords);
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

    for (int x = min_bound.x; x <= max_bound.x; ++x) {
      for (int y = min_bound.y; y <= max_bound.y; ++y) {
        Vector3f bc = Barycentric(a, b, c, Vector2i{ x, y });
        if (bc.x < 0 || bc.y < 0 || bc.z < 0) {
          continue;
        }

        float z = DotProduction(bc, float_clip_coords[2]);

        this->Set(x, y, z, color);
      }
    }
  }

  void Triangle(Matrix43f world_coords, Matrix23f triangle_texture_coords, Matrix33f triangle_normale, const Texture& texture) {
    Matrix43f float_clip_coords = viewport_ * projection_ * modelview_ * (world_coords);
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

    for (int x = min_bound.x; x <= max_bound.x; ++x) {
      for (int y = min_bound.y; y <= max_bound.y; ++y) {
        Vector3f bc = Barycentric(a, b, c, Vector2i{ x, y });
        if (bc.x < 0 || bc.y < 0 || bc.z < 0) {
          continue;
        }

        float z_depth = DotProduction(bc, float_clip_coords[2]);
        
        auto normal = triangle_normale * bc;
        float intensity = DotProduction(normal, kLightDirection);
        if (intensity < 0) continue;

        auto texture_coords = triangle_texture_coords * bc;
        TGAColor color = texture.Get(texture_coords.x, texture_coords.y);
        color.r *= intensity;
        color.g *= intensity;
        color.b *= intensity;
        

        this->Set(x, y, z_depth, color);
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
          float z_depth = DotProduction(bc, float_clip_coords[2]);
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

  void SetViewPort(int x, int y, int w, int h) {
    viewport_ = Matrix44f::Identity();

    viewport_[0][3] = x + w / 2.f;
    viewport_[1][3] = y + h / 2.f;
    viewport_[2][3] = zBufferDepth / 2.f;

    viewport_[0][0] = w / 2.f;
    viewport_[1][1] = h / 2.f;
    viewport_[2][2] = zBufferDepth / 2.f;
  }

  void LookAt(Vector3f eye, Vector3f center, Vector3f up) {
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
    modelview_ = Minv*Tr;
  }

public:
  int width_;
  int height_;
  const int zBufferDepth = 255;

  TGAImage screen_buffer_;
  std::vector<float> zbuffer_;

  Matrix44f projection_;
  Matrix44f viewport_;
  Matrix44f modelview_;
};

void DrawModel(Screen& screen, std::string model_name = "obj/african_head.obj", std::string texture_name = "obj/african_head_diffuse.tga")
{
  Model model;
  model.load(model_name);

  std::cout << "Model loaded" << std::endl;
  std::cout << "nverts: " << model.nverts() << std::endl;
  std::cout << "nfaces: " << model.nfaces() << std::endl;

  Texture texture;
  texture.Load(texture_name);

  TGAImage image(width, height, TGAImage::RGB);

  Shader shader(model, texture, screen.viewport_ * screen.projection_ * screen.modelview_);
  for (size_t f = 0; f < model.num_faces(); ++f) {
    auto clip_coords = shader.Vertex(f);
    screen.Triangle(clip_coords, shader);
  }
}

int main(int argc, char** argv)
{
  Screen screen(width, height);

  std::cout << screen.viewport_ << std::endl;
  std::cout << screen.projection_ << std::endl;
  std::cout << screen.modelview_ << std::endl;

  std::cout << (screen.viewport_ * screen.projection_ * screen.modelview_) << std::endl;
 
 // DrawModel(screen, "obj/reconstructed_head.obj");
  DrawModel(screen);

  screen.WriteTgaFile("output.tga");
  return 0;
}

