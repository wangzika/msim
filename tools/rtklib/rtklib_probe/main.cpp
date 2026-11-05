#include "rtklib.h"
#include <fstream>
#include <iomanip>
#include <iostream>
int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) {
    std::cerr << "usage: msim_rtklib_probe OBS NAV [OBS_JSONL]\n";
    return 2;
  }
  obs_t obs = {};
  nav_t nav = {};
  sta_t station = {};
  const int obs_status = readrnxt(argv[1], 1, {0}, {0}, 0.0, "", &obs, &nav, &station);
  std::cout << "obs_status=" << obs_status << " obs_count=" << obs.n
            << " obs_nav_count=" << nav.n << "\n";
  obs_t nav_obs = {};
  nav_t nav_data = {};
  sta_t nav_station = {};
  const int nav_status = readrnxt(argv[2], 1, {0}, {0}, 0.0, "", &nav_obs, &nav_data, &nav_station);
  std::cout << "nav_status=" << nav_status << " gps_ephemerides=" << nav_data.n
            << " gps_systems=" << nav_data.ng << "\n";
  int result = 0;
  if (obs_status < 0 || obs.n <= 0 || nav_status < 0 || nav_data.n <= 0) result = 1;
  if (!result && argc == 4) {
    std::ofstream out(argv[3]);
    if (!out) {
      std::cerr << "cannot create observation JSONL: " << argv[3] << "\n";
      result = 1;
    } else {
      out << std::setprecision(15);
      for (int i = 0; i < obs.n; ++i) {
        char satellite[8] = {};
        satno2id(obs.data[i].sat, satellite);
        int week = 0;
        const double tow = time2gpst(obs.data[i].time, &week);
        for (int frequency = 0; frequency < NFREQ + NEXOBS; ++frequency) {
          if (!obs.data[i].code[frequency]) continue;
          const char* signal = code2obs(obs.data[i].code[frequency]);
          out << "{\"week\":" << week << ",\"tow_s\":" << tow
              << ",\"satellite\":\"" << satellite << "\",\"signal\":\""
              << (signal ? signal : "") << "\",\"slot\":" << frequency
              << ",\"pseudorange_m\":" << obs.data[i].P[frequency]
              << ",\"carrier_phase_cycles\":" << obs.data[i].L[frequency]
              << ",\"doppler_hz\":" << obs.data[i].D[frequency]
              << ",\"cn0_dbhz\":" << obs.data[i].SNR[frequency] * SNR_UNIT
              << ",\"lli\":" << static_cast<int>(obs.data[i].LLI[frequency])
              << "}\n";
        }
      }
    }
  }
  freeobs(&obs);
  freenav(&nav, 0xFF);
  freeobs(&nav_obs);
  freenav(&nav_data, 0xFF);
  return result;
}
