/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/**
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <gazebo_gps_plugin.h>

#include <boost/algorithm/string.hpp>

using namespace std;

namespace gazebo {
GZ_REGISTER_SENSOR_PLUGIN(GpsPlugin)

GpsPlugin::GpsPlugin() : SensorPlugin()
{ }

GpsPlugin::~GpsPlugin()
{
  if (updateSensorConnection_)
    updateSensorConnection_->~Connection();
  parentSensor_.reset();
  world_->Reset();
}

void GpsPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  parentSensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(_parent);

  if (!parentSensor_)
    gzthrow("GpsPlugin requires a GPS Sensor as its parent");

  // Get the root model name
  const string scopedName = _parent->ParentName();
  vector<std::string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                            [](const string& name)
                            { return name.size() == 0; }), end(names_splitted));
  const string rootModelName = names_splitted.front(); // The first element is the name of the root model

  // the second to the last name is the model name
  const string parentSensorModelName = names_splitted.rbegin()[1];

  // store the model name
  model_name_ = names_splitted[0];

  // get gps topic name
  if(_sdf->HasElement("topic")) {
    gps_topic_ = parentSensor_->Topic();
  } else {
    // if not set by parameter, get the topic name from the model name
    gps_topic_ = parentSensorModelName;
    gzwarn << "[gazebo_gps_plugin]: " + names_splitted.front() + "::" + names_splitted.rbegin()[1] +
      " using gps topic \"" << parentSensorModelName << "\"\n";
  }

  // Store the pointer to the world.
  world_ = physics::get_world(parentSensor_->WorldName());

  #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
    last_gps_time_ = world_->SimTime();
    start_time_ = world_->StartTime();
  #else
    last_time_ = world_->GetSimTime();
    last_gps_time_ = world_->GetSimTime();
    start_time_ = world_->GetStartTime();
  #endif

  // Use environment variables if set for home position.
  const char *env_lat = std::getenv("PX4_HOME_LAT");
  const char *env_lon = std::getenv("PX4_HOME_LON");
  const char *env_alt = std::getenv("PX4_HOME_ALT");

  // Get noise param
  if (_sdf->HasElement("gpsNoise")) {
    getSdfParam<bool>(_sdf, "gpsNoise", gps_noise_, gps_noise_);
  } else {
    gps_noise_ = false;
  }

  const bool world_has_origin = checkWorldHomePosition(world_, world_latitude_, world_longitude_, world_altitude_);

  if (env_lat) {
    lat_home_ = std::stod(env_lat) * M_PI / 180.0;
    gzmsg << "Home latitude is set to " << std::stod(env_lat) << ".\n";
  } else if (world_has_origin) {
    lat_home_ = world_latitude_;
    gzmsg << "[gazebo_gps_plugin] Home latitude is set to " << lat_home_ << ".\n";
  } else if(_sdf->HasElement("homeLatitude")) {
    double latitude;
    getSdfParam<double>(_sdf, "homeLatitude", latitude, lat_home_);
    lat_home_ = latitude * M_PI / 180.0;
  }

  if (env_lon) {
    lon_home_ = std::stod(env_lon) * M_PI / 180.0;
    gzmsg << "Home longitude is set to " << std::stod(env_lon) << ".\n";
  } else if (world_has_origin) {
    lon_home_ = world_longitude_;
    gzmsg << "[gazebo_gps_plugin] Home longitude is set to " << lon_home_ << ".\n";
  } else if(_sdf->HasElement("homeLongitude")) {
    double longitude;
    getSdfParam<double>(_sdf, "homeLongitude", longitude, lon_home_);
    lon_home_ = longitude * M_PI / 180.0;
  }

  if (env_alt) {
    alt_home_ = std::stod(env_alt);
    gzmsg << "Home altitude is set to " << alt_home_ << ".\n";
  } else if (world_has_origin) {
    alt_home_ = world_altitude_;
    gzmsg << "[gazebo_gps_plugin] Home altitude is set to " << alt_home_ << ".\n";
  } else if(_sdf->HasElement("homeAltitude")) {
    getSdfParam<double>(_sdf, "homeAltitude", alt_home_, alt_home_);
  }

  // get random walk in XY plane
  if (_sdf->HasElement("gpsXYRandomWalk")) {
    getSdfParam<double>(_sdf, "gpsXYRandomWalk", gps_xy_random_walk_, kDefaultGpsXYRandomWalk);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default random walk in XY plane: "
           << kDefaultGpsXYRandomWalk << "\n";
  }

  // get random walk in Z
  if (_sdf->HasElement("gpsZRandomWalk")) {
    getSdfParam<double>(_sdf, "gpsZRandomWalk", gps_z_random_walk_, kDefaultGpsZRandomWalk);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default random walk in Z: "
           << kDefaultGpsZRandomWalk << "\n";
  }

  // get position noise density in XY plane
  if (_sdf->HasElement("gpsXYNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsXYNoiseDensity", gps_xy_noise_density_, kDefaultGpsXYNoiseDensity);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default position noise density in XY plane: "
           << kDefaultGpsXYNoiseDensity << "\n";
  }

  // get position noise density in Z
  if (_sdf->HasElement("gpsZNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsZNoiseDensity", gps_z_noise_density_, kDefaultGpsZNoiseDensity);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default position noise density in Z: "
           << kDefaultGpsZNoiseDensity << "\n";
  }

  // get velocity noise density in XY plane
  if (_sdf->HasElement("gpsVXYNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsVXYNoiseDensity", gps_vxy_noise_density_, kDefaultGpsVXYNoiseDensity);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default velocity noise density in XY plane: "
           << kDefaultGpsVXYNoiseDensity << "\n";
  }

  // get velocity noise density in Z
  if (_sdf->HasElement("gpsVZNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsVZNoiseDensity", gps_vz_noise_density_, kDefaultGpsVZNoiseDensity);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default velocity noise density in Z: "
           << kDefaultGpsVZNoiseDensity << "\n";
  }

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  }

  // get update rate
  if (_sdf->HasElement("update_rate")) {
    getSdfParam<double>(_sdf, "update_rate", update_rate_, kDefaultUpdateRate);
  } else {
    update_rate_ = kDefaultUpdateRate;
    gzwarn << "[gazebo_gps_plugin] Using default update rate of "
           << kDefaultUpdateRate << "hz \n";
  }
  parentSensor_->SetUpdateRate(update_rate_);

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  parentSensor_->SetActive(false);
  updateSensorConnection_ = parentSensor_->ConnectUpdated(boost::bind(&GpsPlugin::OnSensorUpdate, this));
  parentSensor_->SetActive(true);

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateWorldConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GpsPlugin::OnWorldUpdate, this, _1));

  gravity_W_ = world_->Gravity();

  gps_pub_ = node_handle_->Advertise<sensor_msgs::msgs::SITLGps>("~/" + model_name_ + "/link/" + gps_topic_, 10);

  gps_ros_pub = n.advertise<mavros_msgs::HilGPS>("fake_gps/gps", 1);
  gps_gt_ros_pub = n.advertise<mavros_msgs::HilGPS>("fake_gps/groundtruth", 1);

  gps_sub = n.subscribe("fake_gps/offset", 1, &GpsPlugin::gps_cb, this);
  gps_sub_inc = n.subscribe("fake_gps/offset_inc", 1, &GpsPlugin::gps_cb_inc, this);
  
  spoofing_signal = n.advertise<std_msgs::Bool>("fake_gps/spoofing_active_signal",1);
}

void GpsPlugin::OnWorldUpdate(const common::UpdateInfo& /*_info*/)
{
  // Store the pointer to the model.
  if (model_ == NULL)
#if GAZEBO_MAJOR_VERSION >= 9
    model_ = world_->ModelByName(model_name_);
#else
    model_ = world_->GetModel(model_name_);
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  current_time_ = world_->SimTime();
#else
  current_time_ = world_->GetSimTime();
#endif
  double dt = (current_time_ - last_time_).Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = model_->WorldPose();
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());
#endif
  // Use the model world position for GPS
  ignition::math::Vector3d& pos_W_I = T_W_I.Pos();
  ignition::math::Quaterniond& att_W_I = T_W_I.Rot();

  // Use the models' world position for GPS velocity.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d velocity_current_W = model_->WorldLinearVel();
#else
  ignition::math::Vector3d velocity_current_W = ignitionFromGazeboMath(model_->GetWorldLinearVel());
#endif

  ignition::math::Vector3d velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.Z() = 0;

  // update noise parameters if gps_noise_ is set
  if (gps_noise_) {
    noise_gps_pos.X() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_pos.Y() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_pos.Z() = gps_z_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel.X() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel.Y() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel.Z() = gps_vz_noise_density_ * sqrt(dt) * randn_(rand_);
    random_walk_gps.X() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);
    random_walk_gps.Y() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);
    random_walk_gps.Z() = gps_z_random_walk_ * sqrt(dt) * randn_(rand_);
  }
  else {
    noise_gps_pos.X() = 0.0;
    noise_gps_pos.Y() = 0.0;
    noise_gps_pos.Z() = 0.0;
    noise_gps_vel.X() = 0.0;
    noise_gps_vel.Y() = 0.0;
    noise_gps_vel.Z() = 0.0;
    random_walk_gps.X() = 0.0;
    random_walk_gps.Y() = 0.0;
    random_walk_gps.Z() = 0.0;
  }

  // gps bias integration
  gps_bias.X() += random_walk_gps.X() * dt - gps_bias.X() / gps_corellation_time_;
  gps_bias.Y() += random_walk_gps.Y() * dt - gps_bias.Y() / gps_corellation_time_;
  gps_bias.Z() += random_walk_gps.Z() * dt - gps_bias.Z() / gps_corellation_time_;

  // reproject position with noise into geographic coordinates
  auto pos_with_noise = pos_W_I + noise_gps_pos + gps_bias;
  auto latlon = reproject(pos_with_noise, lat_home_, lon_home_, alt_home_);

  // fill SITLGps msg
  sensor_msgs::msgs::SITLGps gps_msg;

  gps_msg.set_time_usec(current_time_.Double() * 1e6);
  gps_msg.set_time_utc_usec((current_time_.Double() + start_time_.Double()) * 1e6);

  // @note Unfurtonately the Gazebo GpsSensor seems to provide bad readings,
  // starting to drift and leading to global position loss
  // gps_msg.set_latitude_deg(parentSensor_->Latitude().Degree());
  // gps_msg.set_longitude_deg(parentSensor_->Longitude().Degree());
  // gps_msg.set_altitude(parentSensor_->Altitude());
  gps_msg.set_latitude_deg(latlon.first * 180.0 / M_PI + offset_latitude_);
  gps_msg.set_longitude_deg(latlon.second * 180.0 / M_PI + offset_longitude_);
  gps_msg.set_altitude(pos_W_I.Z() + alt_home_ - noise_gps_pos.Z() + gps_bias.Z() + offset_altitude_);

  std_xy = 1.0;
  std_z = 1.0;
  gps_msg.set_eph(std_xy);
  gps_msg.set_epv(std_z);

  gps_msg.set_velocity_east(velocity_current_W.X() + noise_gps_vel.Y());
  gps_msg.set_velocity(velocity_current_W_xy.Length());
  gps_msg.set_velocity_north(velocity_current_W.Y() + noise_gps_vel.X());
  gps_msg.set_velocity_up(velocity_current_W.Z() - noise_gps_vel.Z());

  // add msg to buffer
  gps_delay_buffer.push(gps_msg);

  last_time_ = current_time_;

  mavros_msgs::HilGPS msg;

  msg.geo.latitude = latlon.first * 180.0 / M_PI;
  msg.geo.longitude = latlon.second * 180.0 / M_PI;
  msg.geo.altitude = pos_W_I.Z() + alt_home_ - noise_gps_pos.Z() + gps_bias.Z();

  msg.fix_type = 3;
  msg.eph = std_xy;
  msg.epv = std_z;
  msg.vel = velocity_current_W_xy.Length();
  msg.vn = velocity_current_W.Y() + noise_gps_vel.X();
  msg.ve = velocity_current_W.X() + noise_gps_vel.Y();
  msg.vd = -velocity_current_W.Z() - noise_gps_vel.Z();
  msg.cog;
  msg.satellites_visible = 10;

  gps_gt_ros_pub.publish(msg);

  ros::spinOnce();
}

void GpsPlugin::gps_cb(const geographic_msgs::GeoPoint::ConstPtr& msg)
{
  offset_latitude_ = msg->latitude;
  offset_longitude_ = msg->longitude;
  offset_altitude_ = msg->altitude;

  checking_gps_spoofing_active(offset_latitude_,offset_longitude_,offset_altitude_);
}

void GpsPlugin::gps_cb_inc(const geographic_msgs::GeoPoint::ConstPtr& msg)
{
  offset_latitude_ = offset_latitude_ + msg->latitude;
  offset_longitude_ = offset_longitude_ + msg->longitude;
  offset_altitude_ = offset_altitude_ + msg->altitude;

  checking_gps_spoofing_active(offset_latitude_,offset_longitude_,offset_altitude_);

}

void GpsPlugin::checking_gps_spoofing_active(const double off_lat,const double off_lon,const double off_alt)
{
  if((off_lat + off_lon + off_alt) == 0 )
  {
    spoofing_signal_active_.data = false;
  }
  else
  {
    spoofing_signal_active_.data = true;
  }
}

void GpsPlugin::OnSensorUpdate()
{
  sensor_msgs::msgs::SITLGps gps_msg;
  // apply GPS delay
  if ((current_time_ - last_gps_time_).Double() > 1 / parentSensor_->UpdateRate()) {
    last_gps_time_ = current_time_;

    while (true) {
      gps_msg = gps_delay_buffer.front();
      double gps_current_delay = current_time_.Double() - gps_delay_buffer.front().time_usec() / 1e6f;

      if (gps_delay_buffer.empty()) {
        // abort if buffer is empty already
        break;
      }
      // remove data that is too old or if buffer size is too large
      if (gps_current_delay >= gps_delay) {
        gps_delay_buffer.pop();
      // remove data if buffer too large
      } else if (gps_delay_buffer.size() > gps_buffer_size_max) {
        gps_delay_buffer.pop();
      } else {
        // if we get here, we have good data, stop
        break;
      }
    }
    // publish SITLGps msg at the defined update rate
    gps_pub_->Publish(gps_msg);

    mavros_msgs::HilGPS msg;
    //msg.data = "Helle";
    //ROS_INFO("%s", msg.data.c_str());

    msg.geo.latitude = gps_delay_buffer.front().latitude_deg();
    msg.geo.longitude = gps_delay_buffer.front().longitude_deg();
    msg.geo.altitude = gps_delay_buffer.front().altitude();

    msg.fix_type = 3;
    msg.eph = gps_delay_buffer.front().eph();
    msg.epv = gps_delay_buffer.front().epv();
    msg.vel = gps_delay_buffer.front().velocity();
    msg.vn = gps_delay_buffer.front().velocity_north();
    msg.ve = gps_delay_buffer.front().velocity_east();
    msg.vd = -gps_delay_buffer.front().velocity_up();
    msg.cog;
    msg.satellites_visible = 10;

    gps_ros_pub.publish(msg);
    spoofing_signal.publish(spoofing_signal_active_);

    ros::spinOnce();
  }
}
} // namespace gazebo
