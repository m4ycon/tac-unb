#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GoToDestination : public BT::StatefulActionNode
{
public:
  GoToDestination(const std::string &name,
                  const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr node_ptr);

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  // Method overrides
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {};

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{BT::InputPort<double>("x"),
                         BT::InputPort<double>("y")};
  }

  // Action Client callback
  void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);

private:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr_;
  bool done_flag_;
};

class Authenticated : public BT::ConditionNode
{
public:
  Authenticated(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr node_ptr) : BT::ConditionNode(name, config), node_ptr_(node_ptr)
  {
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_ptr_;
};

class WaitForSample : public BT::SyncActionNode
{
public:
  WaitForSample(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr node_ptr) : BT::SyncActionNode(name, config), node_ptr_(node_ptr)
  {
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_ptr_;
};

class WaitForCollection : public BT::SyncActionNode
{
public:
  WaitForCollection(const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr node_ptr) : BT::SyncActionNode(name, config), node_ptr_(node_ptr)
  {
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_ptr_;
};
