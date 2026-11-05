#include "msim/gnss/urban.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <stdexcept>

namespace msim::gnss {
namespace {
constexpr double kPi = 3.14159265358979323846;

frames::Vec3 add(frames::Vec3 a, frames::Vec3 b) {
  return {a.x+b.x, a.y+b.y, a.z+b.z};
}
frames::Vec3 subtract(frames::Vec3 a, frames::Vec3 b) {
  return {a.x-b.x, a.y-b.y, a.z-b.z};
}
frames::Vec3 scale(frames::Vec3 a, double x) {
  return {a.x*x, a.y*x, a.z*x};
}
double norm(frames::Vec3 a) { return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z); }

bool segment_hits_box(frames::Vec3 start, frames::Vec3 end,
                      const UrbanBox& box, double first = 0.0,
                      double last = 1.0) {
  const double p[] = {start.x, start.y, start.z};
  const double d[] = {end.x-start.x, end.y-start.y, end.z-start.z};
  const double lo[] = {box.min_enu_m.x, box.min_enu_m.y, box.min_enu_m.z};
  const double hi[] = {box.max_enu_m.x, box.max_enu_m.y, box.max_enu_m.z};
  for (int i = 0; i < 3; ++i) {
    if (std::abs(d[i]) < 1e-12) {
      if (p[i] < lo[i] || p[i] > hi[i]) return false;
      continue;
    }
    double a = (lo[i]-p[i])/d[i], b = (hi[i]-p[i])/d[i];
    if (a > b) std::swap(a,b);
    first = std::max(first,a);
    last = std::min(last,b);
    if (first > last) return false;
  }
  return first <= last;
}

std::array<double,3> components(frames::Vec3 a) { return {a.x,a.y,a.z}; }
frames::Vec3 vector(const std::array<double,3>& a) { return {a[0],a[1],a[2]}; }
double triangle(double offset, double chip) {
  return std::max(0.0, 1.0-std::abs(offset)/chip);
}
}  // namespace

bool urban_direct_blocked(frames::Vec3 receiver, frames::Vec3 satellite,
                          const std::vector<UrbanBox>& boxes) {
  const auto direction = subtract(satellite, receiver);
  const double distance = norm(direction);
  if (distance < 1.0) throw std::invalid_argument("urban ray: satellite too close");
  const auto local_end = add(receiver, scale(direction,
      std::min(distance, 150.0)/distance));
  for (const auto& box : boxes) {
    if (segment_hits_box(receiver, local_end, box)) return true;
  }
  return false;
}

std::vector<UrbanReflection> trace_urban_reflections(
    frames::Vec3 receiver, frames::Vec3 satellite,
    const std::vector<UrbanBox>& boxes, double coefficient) {
  if (!std::isfinite(coefficient) || coefficient < 0.0 || coefficient >= 1.0)
    throw std::invalid_argument("urban reflection coefficient must be in [0,1)");
  std::vector<UrbanReflection> result;
  if (coefficient == 0.0) return result;
  const double direct = norm(subtract(satellite, receiver));
  if (direct < 1.0) throw std::invalid_argument("urban ray: satellite too close");
  const auto r = components(receiver), s = components(satellite);
  for (std::size_t index = 0; index < boxes.size(); ++index) {
    const auto lo = components(boxes[index].min_enu_m);
    const auto hi = components(boxes[index].max_enu_m);
    // Four facades and the roof. No ground reflector is assumed.
    for (int face = 0; face < 5; ++face) {
      const int axis = face < 4 ? face/2 : 2;
      const double plane = face == 4 || face%2 == 1 ? hi[axis] : lo[axis];
      const double receiver_side = r[axis]-plane;
      const double satellite_side = s[axis]-plane;
      if (receiver_side*satellite_side <= 0.0 ||
          (face == 4 && receiver_side <= 0.0)) continue;
      std::array<double,3> mirror = r;
      mirror[axis] = 2.0*plane-r[axis];
      const double denominator = s[axis]-mirror[axis];
      if (std::abs(denominator) < 1e-10) continue;
      const double t = (plane-mirror[axis])/denominator;
      if (t <= 0.0 || t >= 1.0) continue;
      std::array<double,3> point{};
      for (int k = 0; k < 3; ++k) point[k] = mirror[k]+t*(s[k]-mirror[k]);
      bool on_face = true;
      for (int k = 0; k < 3; ++k) {
        if (k != axis && (point[k] < lo[k]-1e-6 || point[k] > hi[k]+1e-6))
          on_face = false;
      }
      if (!on_face) continue;
      const auto impact = vector(point);
      bool obstructed = false;
      for (std::size_t other = 0; other < boxes.size(); ++other) {
        if (other == index) continue;
        if (segment_hits_box(receiver, impact, boxes[other], 1e-5, 1.0-1e-5) ||
            segment_hits_box(satellite, impact, boxes[other], 0.0, 1.0-1e-8)) {
          obstructed = true;
          break;
        }
      }
      if (obstructed) continue;
      const double reflected = norm(subtract(satellite, impact))+
                               norm(subtract(impact, receiver));
      const double excess = std::max(0.0, reflected-direct);
      if (excess > 600.0) continue;
      const double incidence = std::abs(s[axis]-point[axis]) /
                               norm(subtract(satellite, impact));
      const double amplitude = std::min(0.8, coefficient *
          std::sqrt(std::max(0.0, incidence)) * direct/reflected);
      if (amplitude > 0.01)
        result.push_back({excess, amplitude, static_cast<int>(index)*5+face});
    }
  }
  std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
    if (a.relative_amplitude != b.relative_amplitude)
      return a.relative_amplitude > b.relative_amplitude;
    return a.face_id < b.face_id;
  });
  return result;
}

UrbanChannelEffect urban_channel_effect(
    const std::vector<UrbanReflection>& reflections, bool direct_visible,
    double wavelength, double chip) {
  if (!std::isfinite(wavelength) || wavelength <= 0.0 ||
      !std::isfinite(chip) || chip <= 0.0)
    throw std::invalid_argument("urban channel: invalid wavelength or chip");
  UrbanChannelEffect effect;
  effect.available = direct_visible || !reflections.empty();
  if (!effect.available || reflections.empty()) return effect;
  const auto& dominant = reflections.front();
  effect.dominant_face_id = dominant.face_id;
  effect.excess_path_m = dominant.excess_path_m;
  effect.relative_amplitude = dominant.relative_amplitude;
  if (!direct_visible) {
    effect.nlos = true;
    effect.code_bias_m = dominant.excess_path_m;
    effect.phase_bias_cycles = dominant.excess_path_m/wavelength+0.5;
    effect.cn0_change_db = 20.0*std::log10(dominant.relative_amplitude);
    return effect;
  }
  std::complex<double> carrier(1.0,0.0);
  const std::size_t count = std::min<std::size_t>(2, reflections.size());
  for (std::size_t i = 0; i < count; ++i) {
    const auto& echo = reflections[i];
    if (echo.excess_path_m > chip*1.5) continue;
    const double phase = kPi-2.0*kPi*echo.excess_path_m/wavelength;
    carrier += std::polar(echo.relative_amplitude, phase);
  }
  effect.phase_bias_cycles = -std::arg(carrier)/(2.0*kPi);
  effect.cn0_change_db = 20.0*std::log10(std::max(0.01,std::abs(carrier)));
  const double half_spacing = 0.05*chip;  // 0.1-chip early/late spacing.
  auto correlator = [&](double offset) {
    std::complex<double> value(triangle(offset,chip),0.0);
    for (std::size_t i = 0; i < count; ++i) {
      const auto& echo = reflections[i];
      if (echo.excess_path_m > chip*1.5) continue;
      const double phase = kPi-2.0*kPi*echo.excess_path_m/wavelength;
      value += std::polar(echo.relative_amplitude,phase)*
               triangle(offset-echo.excess_path_m,chip);
    }
    return value;
  };
  auto discriminator = [&](double offset) {
    return std::norm(correlator(offset+half_spacing))-
           std::norm(correlator(offset-half_spacing));
  };
  double left = -half_spacing, right = half_spacing;
  const double at_left = discriminator(left), at_right = discriminator(right);
  if (at_left*at_right <= 0.0) {
    for (int i = 0; i < 36; ++i) {
      const double middle = 0.5*(left+right);
      if (discriminator(middle)*at_left > 0.0) left = middle;
      else right = middle;
    }
    effect.code_bias_m = 0.5*(left+right);
  }
  return effect;
}
}  // namespace msim::gnss
