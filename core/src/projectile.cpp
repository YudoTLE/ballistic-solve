#include "ballisticsolver/projectile.hpp"

namespace ballisticsolver
{
  Projectile::Projectile(
      const double mass,
      const double area,
      const double drag_coefficient)
      : mass(mass),
        area(area),
        drag_coefficient(drag_coefficient)
  {
  }
}