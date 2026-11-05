// Drive the pinned SignalSim observation core with sampled Gazebo antenna truth.
// No upstream SignalSim source is modified.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "ScenarioData.h"
#include "Coordinate.h"
#include "Rinex.h"
#include "ConstVal.h"
#include "msim/frames/geodesy.hpp"
#include "msim/gnss/urban.hpp"

namespace frames = msim::frames;

static frames::Vec3 enu(const KINEMATIC_INFO& point, const frames::Geodetic& origin) {
  return frames::ecef_to_enu({point.x, point.y, point.z}, origin);
}

static double wavelength(int system, int signal, int slot) {
  double frequency_mhz = 0.0;
  if (system == GpsSystem)
    frequency_mhz = signal == SIGNAL_INDEX_L2C ? 1227.60 : 1575.42;
  else if (system == BdsSystem)
    frequency_mhz = signal == SIGNAL_INDEX_B2I ? 1207.14 : 1561.098;
  else if (system == GalileoSystem)
    frequency_mhz = signal == SIGNAL_INDEX_E5b ? 1207.14 : 1575.42;
  else if (system == GlonassSystem)
    frequency_mhz = signal == SIGNAL_INDEX_G2 ? 1246.0 + slot * 0.4375 :
                                              1602.0 + slot * 0.5625;
  return 299792458.0 / (frequency_mhz * 1e6);
}

static const char* system_name(int system) {
  switch (system) {
    case GpsSystem: return "GPS";
    case GlonassSystem: return "GLONASS";
    case GalileoSystem: return "Galileo";
    case BdsSystem: return "BeiDou";
    default: throw std::runtime_error("unsupported constellation");
  }
}

static const char* rinex_code(int system, int signal) {
  if (system == GpsSystem) return signal == SIGNAL_INDEX_L2C ? "2L" : "1C";
  if (system == GlonassSystem) return signal == SIGNAL_INDEX_G2 ? "2C" : "1C";
  if (system == GalileoSystem) return signal == SIGNAL_INDEX_E5b ? "7Q" : "1C";
  return signal == SIGNAL_INDEX_B2I ? "7I" : "2I";
}

class SampledScenario : public CScenarioData {
 public:
  void set_epoch(int week, int tow_ms, const KINEMATIC_INFO& position) {
    CurTime = {week, tow_ms, 0.0};
    UtcTime = GpsTimeToUtc(CurTime);
    CurPosEcef = position;
    CurPosLla = EcefToLla(position);
    UpdateSatList();
    UpdateSatelliteParam(0);
  }
};

static void set_obs_type(GnssSystem system, unsigned int out[], unsigned int selected) {
  std::memset(out, 0, RINEX_MAX_FREQ * sizeof(unsigned int));
  int count = 0;
  for (int signal = 0; signal < MAX_OBS_NUMBER && count < RINEX_MAX_FREQ; ++signal) {
    if (!(selected & (1u << signal))) continue;
    unsigned int channel = 0;
    if (system == GpsSystem)
      channel = signal == SIGNAL_INDEX_L1CA ? OBS_CHANNEL_GPS_CA :
                signal == SIGNAL_INDEX_L2C ? OBS_CHANNEL_GPS_L2CL : OBS_CHANNEL_Q;
    else if (system == BdsSystem)
      channel = signal >= SIGNAL_INDEX_B1I && signal <= SIGNAL_INDEX_B3I ?
                OBS_CHANNEL_I : OBS_CHANNEL_P;
    else if (system == GalileoSystem)
      channel = signal == SIGNAL_INDEX_E1 ? OBS_CHANNEL_GAL_E1C : OBS_CHANNEL_Q;
    else if (system == GlonassSystem)
      channel = OBS_CHANNEL_GLO_CA;
    out[count++] = (signal << 8) | OBS_TYPE_MASK_ALL | channel;
  }
}

int main(int argc, char** argv) {
  if (argc != 7) {
    std::cerr << "usage: signalsim_observables scenario.json epochs.txt buildings.txt output.obs output.jsonl receiver\n";
    return 2;
  }
  try {
    auto scenario = std::make_unique<SampledScenario>();
    if (scenario->LoadScenarioFile(argv[1]) != 0)
      throw std::runtime_error("SignalSim scenario could not be loaded");
    if (scenario->OutputParam.Format != OutputFormatRinex ||
        scenario->OutputParam.Interval != 200)
      throw std::runtime_error("SignalSim must output 5 Hz RINEX observations");
    std::ifstream input(argv[2]);
    if (!input) throw std::runtime_error("epochs file missing");
    std::ifstream buildings(argv[3]);
    if (!buildings) throw std::runtime_error("building geometry missing");
    double lat_deg, lon_deg, height, reflectivity;
    int retain_nlos;
    if (!(buildings >> lat_deg >> lon_deg >> height >> reflectivity >> retain_nlos) ||
        (retain_nlos != 0 && retain_nlos != 1))
      throw std::runtime_error("building origin malformed");
    const frames::Geodetic origin{lat_deg * 3.14159265358979323846 / 180.0,
                                  lon_deg * 3.14159265358979323846 / 180.0,
                                  height};
    std::vector<msim::gnss::UrbanBox> boxes;
    double x1, y1, z1, x2, y2, z2;
    while (buildings >> x1 >> y1 >> z1 >> x2 >> y2 >> z2)
      boxes.push_back({{x1, y1, z1}, {x2, y2, z2}});
    if (!buildings.eof()) throw std::runtime_error("building geometry malformed");
    FILE* output = std::fopen(argv[4], "w");
    if (!output) throw std::runtime_error("cannot open RINEX output");
    std::ofstream events(argv[5]);
    if (!events || (std::strcmp(argv[6], "base") && std::strcmp(argv[6], "rover")))
      throw std::runtime_error("cannot open event output or invalid receiver");
    RINEX_HEADER header{};
    header.MajorVersion = 3;
    header.MinorVersion = 3;
    header.HeaderFlag = RINEX_HEADER_PGM | RINEX_HEADER_APPROX_POS |
                        RINEX_HEADER_MK_NAME | RINEX_HEADER_SLOT_FREQ;
    std::strncpy(header.Program, "MSIM-SignalSim", sizeof(header.Program));
    std::strncpy(header.MakerName, "MSIM-UAV", sizeof(header.MakerName));
    header.Interval = 0.2;
    set_obs_type(GpsSystem, header.SysObsTypeGps, scenario->OutputParam.FreqSelect[GpsSystem]);
    set_obs_type(BdsSystem, header.SysObsTypeBds, scenario->OutputParam.FreqSelect[BdsSystem]);
    set_obs_type(GalileoSystem, header.SysObsTypeGalileo, scenario->OutputParam.FreqSelect[GalileoSystem]);
    set_obs_type(GlonassSystem, header.SysObsTypeGlonass, scenario->OutputParam.FreqSelect[GlonassSystem]);
    for (int i = 0; i < 24; ++i)
      header.GlonassFreqNumber[i] = scenario->NavData.GetGlonassSlotFreq(i + 1);
    header.GlonassSlotMask = 0xffffff;

    SAT_OBSERVATION observations[155]{};
    int epochs = 0, signals = 0;
    int previous_week = -1, previous_tow = -1;
    int week, tow_ms;
    KINEMATIC_INFO state{};
    bool wrote_header = false;
    while (input >> week >> tow_ms >> state.x >> state.y >> state.z >>
           state.vx >> state.vy >> state.vz) {
      if (week < 0 || tow_ms < 0 || tow_ms >= 604800000 ||
          !std::isfinite(state.x) || !std::isfinite(state.y) ||
          !std::isfinite(state.z))
        throw std::runtime_error("invalid GNSS epoch or antenna coordinate");
      if (epochs && (week - previous_week) * 604800000LL + tow_ms - previous_tow != 200)
        throw std::runtime_error("epochs are not spaced by 200 ms");
      previous_week = week; previous_tow = tow_ms;
      scenario->set_epoch(week, tow_ms, state);
      if (!wrote_header) {
        header.ApproxPos[0] = state.x;
        header.ApproxPos[1] = state.y;
        header.ApproxPos[2] = state.z;
        OutputHeader(output, &header);
        wrote_header = true;
      }
      int count = 0;
      std::ostringstream epoch_events;
      epoch_events << std::setprecision(16);
      bool first_event = true;
      const auto receiver_enu = enu(state, origin);
      for (int system = 0; system < 4; ++system) {
        CSatelliteParam* satellites[63]{};
        const int found = scenario->GetSatelliteParam(static_cast<GnssSystem>(system), satellites);
        for (int i = 0; i < found; ++i) {
          auto* satellite = satellites[i];
          // The pinned FAST_GLIO RTKLIB build supports BeiDou PRNs 1..46.
          // SignalSim's mixed navigation file also contains C59/C60.
          if (system == BdsSystem && satellite->svid > 46) continue;
          const auto satellite_enu = enu(satellite->PosVel, origin);
          const bool direct = !msim::gnss::urban_direct_blocked(
              receiver_enu, satellite_enu, boxes);
          const auto reflections = msim::gnss::trace_urban_reflections(
              receiver_enu, satellite_enu, boxes, reflectivity);
          if (!direct && (!retain_nlos || reflections.empty())) continue;
          auto& obs = observations[count];
          obs = {};
          obs.system = system;
          obs.svid = satellite->svid;
          obs.ValidMask = scenario->OutputParam.FreqSelect[system];
          std::ostringstream satellite_events;
          satellite_events << std::setprecision(16);
          bool first_signal = true;
          for (int signal = 0; signal < MAX_OBS_NUMBER; ++signal) {
            if (!(obs.ValidMask & (1u << signal))) continue;
            obs.PseudoRange[signal] = satellite->GetTravelTime(signal) * LIGHT_SPEED;
            obs.CarrierPhase[signal] = satellite->GetCarrierPhase(signal);
            obs.Doppler[signal] = satellite->GetDoppler(signal);
            const double wave = wavelength(system, signal, satellite->FreqID);
            const auto effect = msim::gnss::urban_channel_effect(reflections, direct, wave);
            obs.PseudoRange[signal] += effect.code_bias_m;
            obs.CarrierPhase[signal] += effect.phase_bias_cycles;
            obs.CN0[signal] = satellite->CN0 / 100.0 + effect.cn0_change_db;
            if (obs.CN0[signal] < 20.0) {
              obs.ValidMask &= ~(1u << signal);
              continue;
            }
            if (!first_signal) satellite_events << ',';
            first_signal = false;
            satellite_events << "{\"system\":\"" << system_name(system)
                << "\",\"prn\":" << satellite->svid
                << ",\"code\":\"" << rinex_code(system, signal)
                << "\",\"frequency_hz\":" << LIGHT_SPEED / wave
                << ",\"pseudorange_m\":" << obs.PseudoRange[signal]
                << ",\"carrier_phase_cycles\":" << obs.CarrierPhase[signal]
                << ",\"doppler_hz\":" << obs.Doppler[signal]
                << ",\"cn0_dbhz\":" << obs.CN0[signal]
                << ",\"lli\":0,\"sat_pos\":[" << satellite->PosVel.x
                << ',' << satellite->PosVel.y << ',' << satellite->PosVel.z
                << "],\"sat_vel\":[" << satellite->PosVel.vx << ','
                << satellite->PosVel.vy << ',' << satellite->PosVel.vz
                << "],\"sat_clock_bias_s\":" << satellite->ClockError
                << ",\"sat_clock_drift_sps\":"
                << (satellite->EphCur ? satellite->EphCur->af1 : 0.0)
                << ",\"azimuth_rad\":" << satellite->Azimuth
                << ",\"elevation_rad\":" << satellite->Elevation
                << ",\"multipath_code_bias_m\":" << effect.code_bias_m
                << ",\"multipath_phase_bias_cycles\":" << effect.phase_bias_cycles
                << ",\"multipath_excess_path_m\":" << effect.excess_path_m
                << ",\"multipath_relative_amplitude\":" << effect.relative_amplitude
                << ",\"multipath_nlos\":" << (effect.nlos ? "true" : "false") << '}';
          }
          if (obs.ValidMask != scenario->OutputParam.FreqSelect[system]) continue;
          if (!first_event) epoch_events << ',';
          first_event = false;
          epoch_events << satellite_events.str();
          for (int signal = 0; signal < MAX_OBS_NUMBER; ++signal)
            if (obs.ValidMask & (1u << signal)) ++signals;
          ++count;
        }
      }
      const auto gpst = GpsTimeToUtc(scenario->CurTime, FALSE);
      OutputObservation(output, gpst, count, observations);
      events << "{\"receiver\":\"" << argv[6] << "\",\"sim_time_ns\":"
             << static_cast<long long>(epochs) * 200000000LL
             << ",\"gps_week\":" << week << ",\"tow_ns\":"
             << static_cast<long long>(tow_ms) * 1000000LL
             << ",\"observations\":[" << epoch_events.str() << "]}\n";
      ++epochs;
    }
    if (!input.eof() || !wrote_header)
      throw std::runtime_error("epochs file malformed or empty");
    std::fclose(output);
    std::cout << "epochs=" << epochs << " signals=" << signals << "\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
