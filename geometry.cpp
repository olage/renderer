#include "geometry.h"

Vector3f Barycentric(const Vector2i& A, const Vector2i& B,
                     const Vector2i& C, const Vector2i& P) {
  Vector3f v1(B.x - A.x, C.x - A.x, A.x - P.x);
  Vector3f v2(B.y - A.y, C.y - A.y, A.y - P.y);
  Vector3f u = CrossProduction(v1, v2);

  if (std::abs(u.z) < 1.f) return{ -1.f, 1.f, 1.f };

  u.x /= u.z;
  u.y /= u.z;
  u.z = 1.f;

  return { 1.f - (u.x + u.y), u.x, u.y };
}