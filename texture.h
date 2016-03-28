#ifndef __TEXTURE_H__
#define __TEXTURE_H__

#include <algorithm>
#include "tgaimage.h"

class Texture {
public:
  bool Load(const std::string& filename);
  TGAColor Get(float x, float y) const;

private:
  TGAImage data_;
};

#endif //__TEXTURE_H__
