#include "texture.h"

bool Texture::Load(const std::string& filename) {
  if (!data_.read_tga_file(filename.c_str())) {
    return false;
  }
  data_.flip_vertically();
  return true;
}

TGAColor Texture::Get(float x, float y) const {
  int tx = x * data_.get_width();
  tx = std::max(tx, 0);
  tx = std::min(tx, data_.get_width() - 1);

  int ty = y * data_.get_height();
  ty = std::max(ty, 0);
  ty = std::min(ty, data_.get_height() - 1);

  return data_.get(tx, ty);
}
