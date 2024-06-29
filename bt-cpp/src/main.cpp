#include <iostream>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace BT;

class Authenticated : public SyncActionNode
{
public:
  Authenticated(const string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
  {
  }

  NodeStatus tick() override
  {
    cout << "Authenticated: " << this->name() << endl;
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    return {};
  }
};

class GoToDestination : public SyncActionNode
{
public:
  GoToDestination(const string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
  {
  }

  NodeStatus tick() override
  {
    auto [x, y] = getDestination();
    cout << "GoToDestination: " << this->name() << " " << "(" << x << ", " << y << ")" << endl;
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    return {InputPort<int>("x"), InputPort<int>("y")};
  }

private:
  pair<int, int> getDestination() {
    auto x = getInput<int>("x");
    auto y = getInput<int>("y");
    return { x.value_or(0), y.value_or(0) };
  }
};

class WaitForCollection : public SyncActionNode
{
public:
  WaitForCollection(const string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
  {
  }

  NodeStatus tick() override
  {
    cout << "WaitForCollection: " << this->name() << endl;
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    return {};
  }
};

class WaitForSample : public SyncActionNode
{
public:
  WaitForSample(const string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
  {
  }

  NodeStatus tick() override
  {
    cout << "WaitForSample: " << this->name() << endl;
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    return {};
  }
};

int main()
{
  BehaviorTreeFactory factory;

  factory.registerNodeType<Authenticated>("Authenticated");
  factory.registerNodeType<GoToDestination>("GoToDestination");
  factory.registerNodeType<WaitForCollection>("WaitForCollection");
  factory.registerNodeType<WaitForSample>("WaitForSample");

  auto tree = factory.createTreeFromFile("./bts/deliverSample.xml");

  auto status = NodeStatus::RUNNING;
  while (status == NodeStatus::RUNNING)
  {
    cout << "tick" << endl;
    status = tree.tickRoot();
  }

  return 0;
}
