#pragma once

#include <vector>

#include "msim/frames/geodesy.hpp"

namespace msim::gnss {

struct UrbanBox {
  frames::Vec3 min_enu_m;
  frames::Vec3 max_enu_m;
};

struct UrbanReflection {
  double excess_path_m = 0.0;
  double relative_amplitude = 0.0;
  int face_id = -1;
};

struct UrbanChannelEffect {
  bool available = true;
  bool nlos = false;
  double code_bias_m = 0.0;
  double phase_bias_cycles = 0.0;
  double cn0_change_db = 0.0;
  double excess_path_m = 0.0;
  double relative_amplitude = 0.0;
  int dominant_face_id = -1;
};

bool urban_direct_blocked(frames::Vec3 receiver_enu_m,
                          frames::Vec3 satellite_enu_m,
                          const std::vector<UrbanBox>& boxes);

std::vector<UrbanReflection> trace_urban_reflections(
    frames::Vec3 receiver_enu_m, frames::Vec3 satellite_enu_m,
    const std::vector<UrbanBox>& boxes, double reflection_coefficient);

UrbanChannelEffect urban_channel_effect(
    const std::vector<UrbanReflection>& reflections, bool direct_visible,
    double wavelength_m, double chip_length_m = 299792458.0 / 1.023e6);

}  // namespace msim::gnss
