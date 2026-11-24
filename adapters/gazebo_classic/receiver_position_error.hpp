#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

namespace msim {
// A position-domain stochastic model. This is not a pseudorange/RF simulator.
// Arrays use E,N,U. Each sample starts from a stationary Gauss-Markov state.
class ReceiverPositionError {
 public:
  using Axes = std::array<double, 3>;
  Axes white_sigma{}, bias_sigma{}, tau{120.,120.,120.};

  Axes variance() const {
    Axes result{};
    for (int i=0;i<3;++i) result[i]=white_sigma[i]*white_sigma[i]+bias_sigma[i]*bias_sigma[i];
    return result;
  }

  template<class Normal> Axes sample(double dt, Normal normal) {
    if (!std::isfinite(dt) || dt<0.) throw std::invalid_argument("Invalid GPS sample interval");
    Axes result{};
    for (int i=0;i<3;++i) {
      if (tau[i]<=0. || !std::isfinite(tau[i]) || !std::isfinite(bias_sigma[i]) || !std::isfinite(white_sigma[i]) || bias_sigma[i]<0. || white_sigma[i]<0.)
        throw std::invalid_argument("Invalid GPS stochastic parameters");
      const double phi=initialized_?std::exp(-dt/tau[i]):0.;
      bias_[i]=phi*bias_[i]+bias_sigma[i]*std::sqrt(1.-phi*phi)*normal();
      result[i]=bias_[i]+white_sigma[i]*normal();
    }
    initialized_=true;
    return result;
  }

 private:
  Axes bias_{};
  bool initialized_=false;
};
} // namespace msim
