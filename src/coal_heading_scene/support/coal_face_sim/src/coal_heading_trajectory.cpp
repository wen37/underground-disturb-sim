// Experiment-only kinematic route; truth remains independently measured by Gazebo.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>
namespace coal_heading {
class HeadingTrajectory : public gazebo::ModelPlugin {
 public:
  ~HeadingTrajectory() override { update_.reset(); subscription_.reset(); }
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model; initial_ = model->WorldPose();
    speed_ = sdf->Get<double>("speed_m_s");
    travel_start_ = sdf->Get<double>("travel_start_s");
    travel_end_ = sdf->Get<double>("travel_end_s");
    if (!(std::isfinite(speed_) && speed_>0 && travel_start_>=0 && travel_end_>travel_start_))
      throw std::runtime_error("Invalid heading trajectory parameters");
    node_ = gazebo_ros::Node::Get(sdf);
    subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/heading/trajectory_start", rclcpp::QoS(1),
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        if (std::isfinite(msg->data) && msg->data>=0) start_.store(msg->data);
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &info) {
        const double start=start_.load();
        if (start<0) return;
        auto pose=initial_;
        pose.Pos().Y() += speed_*std::clamp(info.simTime.Double()-start-travel_start_,0.0,travel_end_-travel_start_);
        model_->SetWorldPose(pose);
      });
  }
 private:
  gazebo::physics::ModelPtr model_;
  ignition::math::Pose3d initial_;
  double speed_,travel_start_,travel_end_;
  std::atomic<double> start_{-1};
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  gazebo::event::ConnectionPtr update_;
};
GZ_REGISTER_MODEL_PLUGIN(HeadingTrajectory)
}  // namespace coal_heading
