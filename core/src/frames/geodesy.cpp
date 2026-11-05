#include "msim/frames/geodesy.hpp"

#include <cmath>
#include <stdexcept>

namespace msim::frames {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kSemiMajorM = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;
constexpr double kEccentricitySquared = kFlattening * (2.0 - kFlattening);
constexpr double kPoleToleranceM = 1e-9;
constexpr int kMaxIterations = 20;
constexpr double kLatitudeToleranceRad = 1e-14;
constexpr double kSemiMinorM = kSemiMajorM * (1.0 - kFlattening);

void validate(const Geodetic& point) {
  if (!std::isfinite(point.latitude_rad) ||
      !std::isfinite(point.longitude_rad) || !std::isfinite(point.height_m) ||
      std::abs(point.latitude_rad) > kPi / 2 ||
      std::abs(point.longitude_rad) > kPi) {
    throw std::invalid_argument(
        "geodesy.lla: invalid latitude, longitude or height");
  }
}
void validate(const Vec3& point) {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z)) {
    throw std::invalid_argument("geodesy.vector: coordinates must be finite");
  }
}
}  // namespace

Vec3 lla_to_ecef(const Geodetic& lla) {
  validate(lla);
  const double slat = std::sin(lla.latitude_rad);
  const double clat = std::cos(lla.latitude_rad);
  const double radius =
      kSemiMajorM / std::sqrt(1 - kEccentricitySquared * slat * slat);
  return {(radius + lla.height_m) * clat * std::cos(lla.longitude_rad),
          (radius + lla.height_m) * clat * std::sin(lla.longitude_rad),
          (radius * (1 - kEccentricitySquared) + lla.height_m) * slat};
}
Geodetic ecef_to_lla(const Vec3& point) {
  validate(point);
  const double horizontal = std::hypot(point.x, point.y);
  if (horizontal == 0 && point.z == 0) {
    throw std::invalid_argument("geodesy.ecef: earth center is undefined");
  }
  if (horizontal < kPoleToleranceM) {
    return {std::copysign(kPi / 2, point.z), 0,
            std::abs(point.z) - kSemiMinorM};
  }
  double latitude =
      std::atan2(point.z, horizontal * (1 - kEccentricitySquared));
  bool converged = false;
  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    const double sine = std::sin(latitude);
    const double radius =
        kSemiMajorM / std::sqrt(1 - kEccentricitySquared * sine * sine);
    const double updated =
        std::atan2(point.z + kEccentricitySquared * radius * sine, horizontal);
    if (std::abs(updated - latitude) < kLatitudeToleranceRad) {
      latitude = updated;
      converged = true;
      break;
    }
    latitude = updated;
  }
  if (!converged) {
    throw std::runtime_error("geodesy.ecef: conversion did not converge");
  }
  const double sine = std::sin(latitude);
  const double height =
      horizontal * std::cos(latitude) + point.z * sine -
      kSemiMajorM * std::sqrt(1 - kEccentricitySquared * sine * sine);
  return {latitude, std::atan2(point.y, point.x), height};
}
Vec3 ecef_to_enu(const Vec3& point, const Geodetic& origin) {
  validate(point);
  const auto ref = lla_to_ecef(origin);
  const double delta_x = point.x - ref.x;
  const double delta_y = point.y - ref.y;
  const double delta_z = point.z - ref.z;
  const double slat = std::sin(origin.latitude_rad);
  const double clat = std::cos(origin.latitude_rad);
  const double slon = std::sin(origin.longitude_rad);
  const double clon = std::cos(origin.longitude_rad);
  return {-slon * delta_x + clon * delta_y,
          -slat * clon * delta_x - slat * slon * delta_y + clat * delta_z,
          clat * clon * delta_x + clat * slon * delta_y + slat * delta_z};
}
Vec3 enu_to_ecef(const Vec3& enu, const Geodetic& origin) {
  validate(enu);
  const auto ref = lla_to_ecef(origin);
  const double slat = std::sin(origin.latitude_rad);
  const double clat = std::cos(origin.latitude_rad);
  const double slon = std::sin(origin.longitude_rad);
  const double clon = std::cos(origin.longitude_rad);
  return {ref.x - slon * enu.x - slat * clon * enu.y + clat * clon * enu.z,
          ref.y + clon * enu.x - slat * slon * enu.y + clat * slon * enu.z,
          ref.z + clat * enu.y + slat * enu.z};
}
}  // namespace msim::frames
