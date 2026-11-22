// Uniform-beam research LiDAR. Rolling mode casts azimuth batches at different
// physics times, keeping each return in its acquisition-time sensor frame.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <std_msgs/String.h>
#include <cmath>
#include <cstring>
#include <random>
#include <sstream>
#include <vector>
#include <fstream>
#include <algorithm>

namespace msim {
class ScanningLidar : public gazebo::ModelPlugin {
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::MultiRayShapePtr rays_;
  gazebo::physics::CollisionPtr collision_;
  gazebo::event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> ros_;
  ros::Publisher points_, metadata_, livox_;
  ignition::math::Pose3d mounting_;
  std::mt19937 random_;
  std::normal_distribution<double> noise_{0,1};
  unsigned columns_, lines_, column_=0, seq_=0;
  double rate_, min_, max_, sigma_, resolution_, vmin_, vmax_, start_=-1, last_=-1;
  bool rolling_;
  bool livox_pattern_=false;
  std::vector<ignition::math::Vector3d> pattern_;
  size_t pattern_index_=0;
  sensor_msgs::PointCloud2 cloud_;
  size_t capacity_=0;

  void clear() { cloud_.data.clear(); cloud_.width=0; column_=0; }
  void point(float x,float y,float z,float intensity,uint16_t ring,float time) {
    size_t n=cloud_.data.size(); cloud_.data.resize(n+32,0);
    auto p=cloud_.data.data()+n;
    std::memcpy(p,&x,4);std::memcpy(p+4,&y,4);std::memcpy(p+8,&z,4);
    std::memcpy(p+16,&intensity,4);std::memcpy(p+20,&ring,2);std::memcpy(p+24,&time,4);
    ++cloud_.width;
  }
  void cast(unsigned count,double now) {
    // Attached ODE multiray updates link-relative endpoints with the current
    // link pose on every batch. Standalone ODE multiray is not used because
    // Gazebo 11's collision callback does not store standalone hit lengths.
    unsigned needed=count*lines_;
    while(capacity_<needed) { rays_->AddRay({0,0,0},{0,0,1});++capacity_; }
    std::vector<ignition::math::Vector3d> directions; directions.reserve(needed);
    for(unsigned c=0;c<count;++c) for(unsigned l=0;l<lines_;++l) {
      double az=-M_PI+2*M_PI*(column_+c)/columns_;
      double el=vmin_+(vmax_-vmin_)*l/(lines_-1);
      ignition::math::Vector3d dir(cos(el)*cos(az),cos(el)*sin(az),sin(el));
      if(livox_pattern_) dir=pattern_[(pattern_index_+(column_+c)*lines_+l)%pattern_.size()];
      auto w=mounting_.Rot()*dir;
      rays_->SetRay(directions.size(),mounting_.Pos()+min_*w,mounting_.Pos()+max_*w);
      directions.push_back(dir);
    }
    // Disable unused rays from a larger prior batch without extending the scan.
    for(size_t i=needed;i<capacity_;++i) rays_->SetRay(i,mounting_.Pos()+ignition::math::Vector3d(min_,0,0),mounting_.Pos()+ignition::math::Vector3d(max_,0,0));
    rays_->Update();
    float offset=rolling_?static_cast<float>(now-start_):1.e-6f;
    for(unsigned c=0;c<count;++c) for(unsigned l=0;l<lines_;++l) {
      size_t i=c*lines_+l;
      double range=rays_->GetRange(i);
      if(!std::isfinite(range) || range>=max_-1e-6 || range<=min_) continue;
      range+=sigma_*noise_(random_);
      range=std::round(range/resolution_)*resolution_;
      if(range<=min_ || range>=max_) continue;
      auto p=range*directions[i];
      point(p.X(),p.Y(),p.Z(),rays_->GetRetro(i),l,offset);
    }
    column_+=count;
  }
  void publish(double now) {
    // Ring-major packing keeps index-stride filtering from retaining only
    // every fourth ring when point_filter_num=4. Acquisition times are intact.
    if(!livox_pattern_) {
    std::vector<size_t> counts(lines_,0), offsets(lines_,0);
    for(size_t i=0;i<cloud_.data.size();i+=32) {
      uint16_t ring;std::memcpy(&ring,cloud_.data.data()+i+20,2);++counts[ring];
    }
    for(size_t l=1;l<lines_;++l) offsets[l]=offsets[l-1]+counts[l-1];
    std::vector<uint8_t> packed(cloud_.data.size());
    for(size_t i=0;i<cloud_.data.size();i+=32) {
      uint16_t ring;std::memcpy(&ring,cloud_.data.data()+i+20,2);
      std::memcpy(packed.data()+32*offsets[ring]++,cloud_.data.data()+i,32);
    }
    cloud_.data.swap(packed);
    }
    cloud_.header.seq=++seq_;cloud_.header.stamp.fromSec(start_);
    cloud_.row_step=cloud_.width*cloud_.point_step;
    points_.publish(cloud_);
    if(livox_pattern_) {
      livox_ros_driver::CustomMsg msg;
      msg.header=cloud_.header;msg.timebase=msg.header.stamp.toNSec();msg.point_num=cloud_.width;
      msg.points.reserve(msg.point_num);
      for(size_t i=0;i<cloud_.data.size();i+=32) {
        auto p=cloud_.data.data()+i;
        livox_ros_driver::CustomPoint point;
        float intensity,offset;uint16_t ring;
        std::memcpy(&point.x,p,4);std::memcpy(&point.y,p+4,4);std::memcpy(&point.z,p+8,4);
        std::memcpy(&intensity,p+16,4);std::memcpy(&ring,p+20,2);std::memcpy(&offset,p+24,4);
        point.offset_time=std::llround(static_cast<double>(offset)*1e9);
        point.reflectivity=static_cast<uint8_t>(std::clamp(intensity,0.f,255.f));
        point.line=ring;point.tag=0x10;msg.points.push_back(point);
      }
      livox_.publish(msg);
      pattern_index_=(pattern_index_+columns_*lines_)%pattern_.size();
    }
    std_msgs::String m;
    std::ostringstream s;
    s<<"{\"sequence\":"<<seq_<<",\"mode\":\""<<(rolling_?"rolling":"snapshot")
     <<"\",\"pattern\":\""<<(livox_pattern_?"livox_csv":"uniform")<<"\",\"pattern_next_index\":"<<pattern_index_
     <<",\"frame_start_ns\":"<<cloud_.header.stamp.toNSec()
     <<",\"frame_end_ns\":"<<static_cast<uint64_t>(std::llround(now*1e9))
     <<",\"columns\":"<<column_<<",\"points\":"<<cloud_.width<<"}";
    m.data=s.str();metadata_.publish(m);
  }
  void update() {
    double now=world_->SimTime().Double();
    if(last_>=0 && now<last_) { clear();start_=-1;pattern_index_=0; }
    last_=now;
    if(start_<0) { start_=now;clear(); }
    if(!rolling_) {
      if(now-start_+1e-9<1/rate_ && seq_>0) return;
      start_=now;clear();cast(columns_,now);publish(now);return;
    }
    if(now-start_+1e-9>=1/rate_) {
      if(column_!=columns_) {
        ROS_ERROR_THROTTLE(2.,"LiDAR physics steps too coarse for configured scan; dropping incomplete frame");
      } else publish(now);
      start_=now;clear();
    }
    // Quantize acquisition to real physics steps, not invented per-point times.
    double step=world_->Physics()->GetMaxStepSize();
    unsigned due=std::min(columns_,static_cast<unsigned>(std::ceil((now-start_+step)*rate_*columns_-1e-7)));
    if(due>column_) cast(due-column_,now);
  }
 public:
  ~ScanningLidar() override { update_.reset(); }
  void Load(gazebo::physics::ModelPtr model,sdf::ElementPtr s) override {
    if(!ros::isInitialized()) throw std::runtime_error("LiDAR needs Gazebo ROS API plugin");
    world_=model->GetWorld();link_=model->GetLink("base_link");
    columns_=s->Get<unsigned>("horizontal_samples");lines_=s->Get<unsigned>("vertical_samples");
    rate_=s->Get<double>("rate_hz");min_=s->Get<double>("range_min_m");max_=s->Get<double>("range_max_m");
    sigma_=s->Get<double>("range_noise_sigma_m");resolution_=s->Get<double>("range_resolution_m");vmin_=s->Get<double>("vertical_min_deg")*M_PI/180.;vmax_=s->Get<double>("vertical_max_deg")*M_PI/180.;
    mounting_=s->Get<ignition::math::Pose3d>("pose"); rolling_=s->Get<std::string>("timing")=="rolling";
    livox_pattern_=s->HasElement("scan_pattern") && s->Get<std::string>("scan_pattern")=="livox_csv";
    if(livox_pattern_) {
      std::ifstream file(s->Get<std::string>("pattern_file"));
      if(!file) throw std::runtime_error("Livox pattern file cannot be opened");
      std::string line;std::getline(file,line);size_t index=0;
      while(std::getline(file,line)) {
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream row(line);double sample,az,zen;
        if(!(row>>sample>>az>>zen) || !std::isfinite(az) || !std::isfinite(zen) || sample!=index+1)
          throw std::runtime_error("Invalid or non-contiguous Livox scan pattern");
        ++index;az*=M_PI/180.;double el=M_PI/2-zen*M_PI/180.;
        pattern_.emplace_back(cos(el)*cos(az),cos(el)*sin(az),sin(el));
      }
      if(pattern_.size()<columns_*lines_ || pattern_.size()%lines_!=0)
        throw std::runtime_error("Livox pattern is too short or not channel-aligned");
    }
    random_.seed(s->Get<unsigned>("seed"));
    collision_=world_->Physics()->CreateCollision("multiray",link_);
    collision_->SetName("msim_lidar_collision");collision_->SetRelativePose(mounting_);
    rays_=boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(collision_->GetShape());
    if(!rays_) throw std::runtime_error("LiDAR requires a multiray-capable physics engine");
    rays_->Load(s);rays_->Init();capacity_=rays_->RayCount();
    cloud_.header.frame_id="velodyne";cloud_.height=1;cloud_.point_step=32;cloud_.is_dense=true;
    const char* names[]={"x","y","z","intensity","ring","time"};
    unsigned offsets[]={0,4,8,16,20,24};
    for(unsigned i=0;i<6;++i) {
      sensor_msgs::PointField f;f.name=names[i];f.offset=offsets[i];f.count=1;
      f.datatype=i==4?sensor_msgs::PointField::UINT16:sensor_msgs::PointField::FLOAT32;
      cloud_.fields.push_back(f);
    }
    ros_.reset(new ros::NodeHandle("msim_lidar"));points_=ros_->advertise<sensor_msgs::PointCloud2>("/velodyne_points",3);
    if(livox_pattern_) livox_=ros_->advertise<livox_ros_driver::CustomMsg>("/livox/lidar",3);
    metadata_=ros_->advertise<std_msgs::String>("/msim/lidar_scan_info",3);
    update_=gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&ScanningLidar::update,this));
    ROS_INFO("msim LiDAR %u x %u, %.1f Hz, %s, radial noise %.3f m",columns_,lines_,rate_,rolling_?"rolling":"snapshot",sigma_);
  }
};
GZ_REGISTER_MODEL_PLUGIN(ScanningLidar)
} // namespace msim
