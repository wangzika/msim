#include "msim/frames/geodesy.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

// NOLINTBEGIN(readability-magic-numbers)
int main() {
  using namespace msim::frames;
  constexpr double kPi = 3.14159265358979323846;
  const Geodetic origin{37.352721 * kPi / 180, -121.915773 * kPi / 180, 20};
  const auto actual = lla_to_ecef(origin);
  // Independent reference: PROJ cs2cs, WGS84, 2026-10-29.
  const Vec3 expected{-2683726.736798946, -4308942.772727888,
                      3848595.419222301};
  const auto distance = [](const Vec3& lhs, const Vec3& rhs) {
    return std::hypot(std::hypot(lhs.x - rhs.x, lhs.y - rhs.y), lhs.z - rhs.z);
  };
  if (distance(actual, expected) > 1e-4) {
    return 1;
  }
  for (const auto& point : std::array<Geodetic, 5>{{{0, 0, 0},
                                                    {kPi / 2, 0, 50},
                                                    {-kPi / 2, kPi, 30},
                                                    {0.7, kPi - 1e-10, 100},
                                                    origin}}) {
    const auto ecef = lla_to_ecef(point);
    if (distance(ecef, lla_to_ecef(ecef_to_lla(ecef))) > 1e-4) {
      return 2;
    }
  }
  const Vec3 local{12.5, -30.2, 5.75};
  if (distance(local, ecef_to_enu(enu_to_ecef(local, origin), origin)) > 1e-8) {
    return 3;
  }
  try {
    static_cast<void>(ecef_to_lla({0, 0, 0}));
    return 4;
  } catch (const std::invalid_argument&) {  // NOLINT(bugprone-empty-catch)
    // Expected.
  }
  return 0;
}
// NOLINTEND(readability-magic-numbers)
