#pragma once

namespace msim::frames {
struct Vec3 {
  double x;
  double y;
  double z;
};
struct Geodetic {
  double latitude_rad;
  double longitude_rad;
  double height_m;
};

// WGS84 ellipsoid; input angles are radians and distances metres.
Vec3 lla_to_ecef(const Geodetic& lla);
Geodetic ecef_to_lla(const Vec3& point);
Vec3 ecef_to_enu(const Vec3& point_ecef_m, const Geodetic& origin);
Vec3 enu_to_ecef(const Vec3& enu_m, const Geodetic& origin);
}  // namespace msim::frames
