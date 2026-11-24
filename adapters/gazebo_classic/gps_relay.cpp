// Preserve the PX4 native GPS sample (including its noise and delay), then
// apply one receiver/attack realization for both PX4 and ROS consumers.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/transport/transport.hh>
#include <SITLGps.pb.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <array>
#include <deque>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include "receiver_position_error.hpp"

namespace msim {
using Vec = ignition::math::Vector3d;
constexpr double A = 6378137., E2 = 6.69437999014e-3;
double env(const char* name, double fallback) {
  const char* s = std::getenv(name); return s ? std::stod(s) : fallback;
}
Vec ecef(double lat, double lon, double alt) {
  lat *= M_PI/180.; lon *= M_PI/180.;
  double n = A / std::sqrt(1-E2*std::sin(lat)*std::sin(lat));
  return {(n+alt)*std::cos(lat)*std::cos(lon),
          (n+alt)*std::cos(lat)*std::sin(lon), (n*(1-E2)+alt)*std::sin(lat)};
}
Vec lla(const Vec& v) {
  double lon = std::atan2(v.Y(),v.X()), h = 0, p = std::hypot(v.X(),v.Y());
  double lat = std::atan2(v.Z(),p*(1-E2));
  for (int i=0; i<10; ++i) {
    double n=A/std::sqrt(1-E2*std::sin(lat)*std::sin(lat));
    h=p/std::cos(lat)-n;
    lat=std::atan2(v.Z(),p*(1-E2*n/(n+h)));
  }
  return {lat*180/M_PI,lon*180/M_PI,h};
}

class GpsRelay : public gazebo::ModelPlugin {
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::RayShapePtr sky_ray_;
  ignition::math::Pose3d antenna_pose_;
  gazebo::transport::NodePtr gz_;
  gazebo::transport::SubscriberPtr sub_;
  gazebo::transport::PublisherPtr output_;
  gazebo::event::ConnectionPtr update_;
  std::deque<boost::shared_ptr<const sensor_msgs::msgs::SITLGps>> pending_;
  std::unique_ptr<ros::NodeHandle> ros_;
  ros::CallbackQueue queue_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  ros::Publisher normal_pub_, attack_pub_, samples_pub_;
  ros::Subscriber command_;
  Vec origin_, origin_ecef_;
  ignition::math::Matrix3d enu_;
  std::mt19937 random_;
  std::normal_distribution<double> gaussian_{0,1};
  double ps_xy_, ps_z_, vs_xy_, vs_z_, var_xy_, var_z_;
  double bias_xy_, bias_z_, bias_tau_;
  ReceiverPositionError position_error_;
  bool per_axis_error_=false, model_variance_=false;
  std::string position_model_id_="legacy", environment_model_="disabled";
  bool adaptive_environment_=false;
  double environment_range_=80., environment_gain_=3.;
  Vec bias_, fixed_offset_;
  double first_=-1, last_=-1;
  uint32_t seq_=0;
  struct Attack {double start, east_rate, north_rate; Vec offset; bool velocity;};
  Attack attack_;
  std::mutex mutex_;

  Vec to_enu(double lat, double lon, double alt) const {
    return enu_*(ecef(lat,lon,alt)-origin_ecef_);
  }
  Vec to_lla(const Vec& p) const { return lla(origin_ecef_+enu_.Transposed()*p); }

  // A fixed set of upward rays is a reproducible proxy for local sky visibility.
  // It uses collision geometry, so mesh scenes and changing obstacles contribute.
  double blocked_sky_fraction() {
    if (!adaptive_environment_) return 0.;
    const Vec antenna=(model_->WorldPose()*antenna_pose_).Pos();
    constexpr int azimuths=12;
    const double elevations[]={15.,35.,60.};
    int blocked=0;
    for (double elevation : elevations) for (int i=0;i<azimuths;++i) {
      double az=2.*M_PI*i/azimuths, el=elevation*M_PI/180.;
      Vec direction(cos(el)*cos(az),cos(el)*sin(az),sin(el));
      // Start outside the Iris body; a self-hit is skipped if another model is used.
      Vec start=antenna+direction*.6;
      bool hit=false;
      for (int attempt=0;attempt<4;++attempt) {
        sky_ray_->SetPoints(start,antenna+direction*environment_range_);
        double distance=environment_range_;
        std::string entity;
        sky_ray_->GetIntersection(distance,entity);
        if (entity.empty() || !std::isfinite(distance) || distance>=environment_range_) break;
        const std::string own=model_->GetName()+"::";
        const std::string scoped=model_->GetScopedName()+"::";
        if (entity.compare(0,own.size(),own)!=0 &&
            entity.compare(0,scoped.size(),scoped)!=0) {
          hit=true; break;
        }
        start+=direction*(std::max(0.,distance)+.1);
      }
      if (hit) ++blocked;
    }
    return static_cast<double>(blocked)/(azimuths*3);
  }

  nav_msgs::Odometry odom(const Vec& p, const Vec& v, double stamp,
                          double variance_e, double variance_n, double variance_z,
                          double noise_scale) {
    nav_msgs::Odometry m;
    m.header.seq=seq_; m.header.stamp.fromSec(stamp);
    m.header.frame_id="msim_enu"; m.child_frame_id="gnss_antenna";
    m.pose.pose.position.x=p.X(); m.pose.pose.position.y=p.Y(); m.pose.pose.position.z=p.Z();
    m.pose.pose.orientation.w=1.;
    m.pose.covariance[0]=variance_e; m.pose.covariance[7]=variance_n; m.pose.covariance[14]=variance_z;
    m.twist.twist.linear.x=v.X(); m.twist.twist.linear.y=v.Y(); m.twist.twist.linear.z=v.Z();
    m.twist.covariance[0]=m.twist.covariance[7]=vs_xy_*vs_xy_*noise_scale*noise_scale;
    m.twist.covariance[14]=vs_z_*vs_z_*noise_scale*noise_scale;
    return m;
  }

  void control(const std_msgs::String::ConstPtr& msg) {
    try {
      boost::property_tree::ptree t; std::istringstream s(msg->data);
      boost::property_tree::read_json(s,t);
      std::lock_guard<std::mutex> lock(mutex_);
      Attack a=attack_;
      if (!t.get<bool>("enabled",true)) {
        a.start=std::numeric_limits<double>::infinity(); a.offset=Vec::Zero;
      } else {
        a.start=std::max(0.,world_->SimTime().Double()-(first_<0?0:first_)) + t.get<double>("start_delay_s",0.);
        a.east_rate=t.get<double>("east_rate_mps",0.);
        a.north_rate=t.get<double>("north_rate_mps",0.);
        a.offset={t.get<double>("east_offset_m",0.),t.get<double>("north_offset_m",0.),t.get<double>("up_offset_m",0.)};
        a.velocity=t.get<bool>("velocity_consistent",true);
        if (!std::isfinite(a.start) || !std::isfinite(a.east_rate) || !std::isfinite(a.north_rate) ||
            !a.offset.IsFinite() || std::abs(a.east_rate)>10 || std::abs(a.north_rate)>10 || a.offset.Length()>1000)
          throw std::runtime_error("invalid attack parameters");
      }
      attack_=a;
    } catch (const std::exception& e) { ROS_ERROR("GPS attack command rejected: %s",e.what()); }
  }

  void on_gps(const boost::shared_ptr<const sensor_msgs::msgs::SITLGps>& msg) {
    // Gazebo Transport invokes this on a network worker. ODE ray casting and
    // model/world access must stay on the physics update thread, including when
    // the scene contains triangle meshes. Preserve the native measurement time.
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_.size()>=32) pending_.pop_front();
    pending_.push_back(msg);
  }

  void update() {
    std::deque<boost::shared_ptr<const sensor_msgs::msgs::SITLGps>> pending;
    {std::lock_guard<std::mutex> lock(mutex_);pending.swap(pending_);}
    for (const auto& msg : pending) process_gps(msg);
  }

  void process_gps(const boost::shared_ptr<const sensor_msgs::msgs::SITLGps>& msg) {
    const double stamp=msg->time_usec()*1e-6;
    double delta=last_<0?0:stamp-last_;
    Attack a;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stamp<=last_) return;
      last_=stamp; if (first_<0) first_=stamp;
      a=attack_;
    }
    ++seq_;
    Vec p=to_enu(msg->latitude_deg(),msg->longitude_deg(),msg->altitude());
    const double blocked=blocked_sky_fraction();
    const double noise_scale=1.+environment_gain_*blocked;
    const auto marginal=position_error_.variance();
    const double variance_e=(model_variance_?marginal[0]:var_xy_)*noise_scale*noise_scale;
    const double variance_n=(model_variance_?marginal[1]:var_xy_)*noise_scale*noise_scale;
    const double variance_xy=std::max(variance_e,variance_n);
    const double variance_z=(model_variance_?marginal[2]:var_z_)*noise_scale*noise_scale;
    if (per_axis_error_) {
      const auto error=position_error_.sample(delta,[this](){return gaussian_(random_);});
      p+=noise_scale*Vec(error[0],error[1],error[2]);
    } else {
      // Legacy realization retained for historical experimental presets.
      double phi=seq_==1?0:std::exp(-delta/bias_tau_);
      double gain=std::sqrt(1-phi*phi);
      bias_=phi*bias_+gain*Vec(bias_xy_*gaussian_(random_),bias_xy_*gaussian_(random_),bias_z_*gaussian_(random_));
      p+=noise_scale*bias_;
      p += Vec(noise_scale*ps_xy_*gaussian_(random_),noise_scale*ps_xy_*gaussian_(random_),noise_scale*ps_z_*gaussian_(random_));
    }
    Vec v(msg->velocity_east()+noise_scale*vs_xy_*gaussian_(random_),
          msg->velocity_north()+noise_scale*vs_xy_*gaussian_(random_),
          msg->velocity_up()+noise_scale*vs_z_*gaussian_(random_));
    bool active=stamp-first_>=a.start;
    double dt=active?stamp-first_-a.start:0.;
    Vec shift=fixed_offset_+(active?a.offset+Vec(dt*a.east_rate,dt*a.north_rate,0):Vec::Zero);
    Vec pv=p+shift, vv=v;
    if(active && a.velocity) vv+=Vec(a.east_rate,a.north_rate,0);
    Vec ll=to_lla(pv);
    sensor_msgs::msgs::SITLGps attacked=*msg;
    // Do not perturb native LLA fields by a round trip in the exact pass-through case.
    if(per_axis_error_ || ps_xy_!=0 || ps_z_!=0 || bias_xy_!=0 || bias_z_!=0 || shift.Length()!=0) {
      attacked.set_latitude_deg(ll.X()); attacked.set_longitude_deg(ll.Y()); attacked.set_altitude(ll.Z());
    }
    attacked.set_velocity_east(vv.X()); attacked.set_velocity_north(vv.Y()); attacked.set_velocity_up(vv.Z());
    attacked.set_velocity(std::hypot(vv.X(),vv.Y()));
    // Model-matched position-domain variance, or legacy experimental weights.
    // Neither is a claim of calibrated uncertainty for the upstream native sensor.
    attacked.set_eph(std::sqrt(variance_xy)); attacked.set_epv(std::sqrt(variance_z));
    output_->Publish(attacked);
    normal_pub_.publish(odom(p,v,stamp,variance_e,variance_n,variance_z,noise_scale));
    attack_pub_.publish(odom(pv,vv,stamp,variance_e,variance_n,variance_z,noise_scale));
    std::ostringstream j; j<<std::setprecision(16)
      <<"{\"source\":\"gazebo_gps_plugin\",\"sequence\":"<<seq_
      <<",\"sim_time_ns\":"<<msg->time_usec()*1000
      <<",\"gnss_origin_sim_time_ns\":"<<static_cast<uint64_t>(std::llround(first_*1e9))
      <<",\"publish_time_ns\":"<<world_->SimTime().nsec+world_->SimTime().sec*1000000000LL
      <<",\"lat_deg\":"<<attacked.latitude_deg()<<",\"lon_deg\":"<<attacked.longitude_deg()<<",\"alt_m\":"<<attacked.altitude()
      <<",\"normal_east_m\":"<<p.X()<<",\"normal_north_m\":"<<p.Y()<<",\"normal_up_m\":"<<p.Z()
      <<",\"gps_east_m\":"<<pv.X()<<",\"gps_north_m\":"<<pv.Y()<<",\"gps_up_m\":"<<pv.Z()
      <<",\"attack_east_m\":"<<shift.X()<<",\"attack_north_m\":"<<shift.Y()<<",\"attack_up_m\":"<<shift.Z()
      <<",\"attack_active\":"<<(active?"true":"false")
      <<",\"sky_blocked_fraction\":"<<blocked
      <<",\"environment_noise_scale\":"<<noise_scale
      <<",\"position_error_model_id\":\""<<position_model_id_<<"\""
      <<",\"environment_model\":\""<<environment_model_<<"\""
      <<",\"reported_variance_mode\":\""<<(model_variance_?"model_marginal":"explicit_experimental")<<"\""
      <<",\"reported_variance_enu_m2\":["<<variance_e<<","<<variance_n<<","<<variance_z<<"]"
      <<",\"reported_variance_xy_m2\":"<<variance_xy
      <<",\"reported_variance_z_m2\":"<<variance_z
      <<",\"velocity_consistent\":"<<(a.velocity?"true":"false")<<"}";
    std_msgs::String status; status.data=j.str(); samples_pub_.publish(status);
  }

 public:
  ~GpsRelay() override { update_.reset();sub_.reset(); if(spinner_) spinner_->stop(); }
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr s) override {
    if(!ros::isInitialized()) throw std::runtime_error("GPS relay needs Gazebo ROS API plugin");
    world_=model->GetWorld();
    model_=model;
    antenna_pose_=s->Get<ignition::math::Pose3d>("antenna_pose");
    adaptive_environment_=s->Get<bool>("adaptive_environment");
    environment_range_=s->Get<double>("environment_range_m");
    environment_gain_=s->Get<double>("environment_noise_gain");
    if (adaptive_environment_) {
      sky_ray_=boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
          world_->Physics()->CreateShape("ray",gazebo::physics::CollisionPtr()));
      if (!sky_ray_) throw std::runtime_error("GPS environment model requires ray casting");
    }
    origin_={env("PX4_HOME_LAT",30.5312),env("PX4_HOME_LON",114.3601),env("PX4_HOME_ALT",50)};
    origin_ecef_=ecef(origin_.X(),origin_.Y(),origin_.Z());
    double lat=origin_.X()*M_PI/180., lon=origin_.Y()*M_PI/180.;
    enu_=ignition::math::Matrix3d(-sin(lon),cos(lon),0,-sin(lat)*cos(lon),-sin(lat)*sin(lon),cos(lat),cos(lat)*cos(lon),cos(lat)*sin(lon),sin(lat));
    ps_xy_=s->Get<double>("position_sigma_xy_m"); ps_z_=s->Get<double>("position_sigma_z_m");
    vs_xy_=s->Get<double>("velocity_sigma_xy_mps"); vs_z_=s->Get<double>("velocity_sigma_z_mps");
    var_xy_=s->Get<double>("reported_variance_xy_m2"); var_z_=s->Get<double>("reported_variance_z_m2");
    bias_xy_=s->Get<double>("bias_sigma_xy_m");bias_z_=s->Get<double>("bias_sigma_z_m");bias_tau_=s->Get<double>("bias_tau_s");
    per_axis_error_=s->HasElement("position_sigma_enu_m");
    if (per_axis_error_) {
      const Vec white=s->Get<Vec>("position_sigma_enu_m"),bias=s->Get<Vec>("bias_sigma_enu_m"),tau=s->Get<Vec>("bias_tau_enu_s");
      position_error_.white_sigma={white.X(),white.Y(),white.Z()};
      position_error_.bias_sigma={bias.X(),bias.Y(),bias.Z()};
      position_error_.tau={tau.X(),tau.Y(),tau.Z()};
    }
    model_variance_=s->HasElement("reported_variance_mode") && s->Get<std::string>("reported_variance_mode")=="model_marginal";
    if (model_variance_ && (!per_axis_error_ || adaptive_environment_ || s->Get<bool>("native_noise")))
      throw std::runtime_error("Model-matched GPS variance requires ENU errors without native noise or empirical environment scaling");
    if(s->HasElement("position_error_model_id")) position_model_id_=s->Get<std::string>("position_error_model_id");
    environment_model_=adaptive_environment_?"uncalibrated_sky_fraction":"disabled";
    fixed_offset_={env("MSIM_GNSS_EAST_BIAS_M",0),env("MSIM_GNSS_NORTH_BIAS_M",0),0};
    random_.seed(s->Get<unsigned>("seed"));
    attack_={env("MSIM_SPOOF_START_S",INFINITY),env("MSIM_SPOOF_EAST_RATE_MPS",0),env("MSIM_SPOOF_NORTH_RATE_MPS",0),
             Vec::Zero,s->Get<bool>("spoof_velocity_consistent")};
    ros_.reset(new ros::NodeHandle("msim_gps")); ros_->setCallbackQueue(&queue_);
    normal_pub_=ros_->advertise<nav_msgs::Odometry>("/msim/normal_gnss_odom",20);
    attack_pub_=ros_->advertise<nav_msgs::Odometry>("/msim/spoofed_gnss_odom",20);
    samples_pub_=ros_->advertise<std_msgs::String>("/msim/gps_samples",20);
    command_=ros_->subscribe("/msim/gps_attack",5,&GpsRelay::control,this);
    gz_.reset(new gazebo::transport::Node()); gz_->Init(world_->Name());
    output_=gz_->Advertise<sensor_msgs::msgs::SITLGps>("~/"+model->GetName()+"/link/gps0",10);
    sub_=gz_->Subscribe("~/"+model->GetName()+"/link/msim_gps_raw",&GpsRelay::on_gps,this);
    update_=gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GpsRelay::update,this));
    spinner_.reset(new ros::AsyncSpinner(1,&queue_)); spinner_->start();
    ROS_INFO_STREAM("msim GPS relay ready: position model="<<position_model_id_<<", environment="<<environment_model_<<", native_noise="<<s->Get<bool>("native_noise"));
  }
};
GZ_REGISTER_MODEL_PLUGIN(GpsRelay)
} // namespace msim
