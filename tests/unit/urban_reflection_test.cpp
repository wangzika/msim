#include <cmath>
#include <stdexcept>
#include <vector>

#include "msim/gnss/urban.hpp"

namespace {
void require(bool condition) {
  if (!condition) throw std::runtime_error("urban reflection contract failed");
}
}  // namespace

int main() {
  using msim::frames::Vec3;
  using msim::gnss::UrbanBox;
  using msim::gnss::urban_channel_effect;
  using msim::gnss::urban_direct_blocked;
  using msim::gnss::trace_urban_reflections;
  const Vec3 receiver{0.0, 0.0, 5.0};
  const Vec3 satellite{-2.0e7, 0.0, 2.0e7};
  const std::vector<UrbanBox> walls{{{5.0,-5.0,0.0},{6.0,5.0,20.0}}};
  require(!urban_direct_blocked(receiver, satellite, walls));
  const auto echoes = trace_urban_reflections(receiver, satellite, walls, 0.35);
  require(!echoes.empty());
  require(echoes.front().excess_path_m > 0.0);
  require(echoes.front().relative_amplitude > 0.0);
  const auto los = urban_channel_effect(echoes, true, 299792458.0/1575.42e6);
  require(los.available && !los.nlos);
  require(std::isfinite(los.code_bias_m) && std::isfinite(los.phase_bias_cycles));
  require(std::abs(los.code_bias_m) > 0.01);
  const auto reflected_only = urban_channel_effect(echoes, false,
                                                   299792458.0/1575.42e6);
  require(reflected_only.available && reflected_only.nlos);
  require(std::abs(reflected_only.code_bias_m-echoes.front().excess_path_m) < 1e-8);
  const std::vector<UrbanBox> blocker{{{-6.0,-5.0,0.0},{-5.0,5.0,20.0}}};
  require(urban_direct_blocked(receiver, satellite, blocker));
  require(!urban_channel_effect({}, false, 299792458.0/1575.42e6).available);
}
