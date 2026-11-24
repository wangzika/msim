#include "receiver_position_error.hpp"
#include <cassert>
#include <iostream>
#include <random>

int main() {
  msim::ReceiverPositionError model;
  model.white_sigma={.03,0.,.02};
  model.bias_sigma={std::sqrt(.08),std::sqrt(.08),std::sqrt(.18)};
  model.tau={128.51,65.6,95.31};
  const auto variance=model.variance();
  assert(std::abs(variance[0]-.0809)<1e-12);
  assert(std::abs(variance[1]-.08)<1e-12);
  assert(std::abs(variance[2]-.1804)<1e-12);
  std::mt19937 generator(20261123);
  std::normal_distribution<double> normal;
  auto draw=[&](){return normal(generator);};
  // Independent first samples must already have the stationary covariance.
  std::array<double,3> initial_sq{};
  for(int n=0;n<30000;++n) {
    auto independent=model;
    const auto x=independent.sample(0.,draw);
    for(int i=0;i<3;++i)initial_sq[i]+=x[i]*x[i]/30000.;
  }
  for(int i=0;i<3;++i)assert(std::abs(initial_sq[i]/variance[i]-1.)<.05);
  // Verify actual generated variance and temporal covariance, at two cadences.
  std::cout<<"{\"passed\":true,\"cadences\":[";
  for(double dt : {.2,1.}) {
    auto series=model;
    constexpr int count=700000;
    std::array<double,3> sum{},square{},cross{},previous{};
    for(int n=0;n<count;++n) {
      const auto x=series.sample(dt,draw);
      for(int i=0;i<3;++i) {
        sum[i]+=x[i];square[i]+=x[i]*x[i];
        if(n)cross[i]+=x[i]*previous[i];
      }
      previous=x;
    }
    if(dt==1.)std::cout<<",";
    std::cout<<"{\"dt_s\":"<<dt<<",\"samples\":"<<count<<",\"axes\":[";
    for(int i=0;i<3;++i) {
      const double mean=sum[i]/count,var=square[i]/count-mean*mean;
      const double corr=(cross[i]/(count-1)-mean*mean)/var;
      const double target_corr=model.bias_sigma[i]*model.bias_sigma[i]*std::exp(-dt/model.tau[i])/variance[i];
      assert(std::abs(mean)<.05);
      assert(std::abs(var/variance[i]-1.)<.12);
      assert(std::abs(corr-target_corr)<.004);
      if(i)std::cout<<",";
      std::cout<<"{\"mean_m\":"<<mean<<",\"variance_m2\":"<<var<<",\"target_variance_m2\":"<<variance[i]
               <<",\"lag_one_correlation\":"<<corr<<",\"target_correlation\":"<<target_corr<<"}";
    }
    std::cout<<"]}";
  }
  bool rejected=false;
  try{model.sample(-1.,draw);}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
  std::cout<<"]}\n";
}
