#include <iostream>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"

using namespace std;
using namespace BT;

class Authenticate : public SyncActionNode
{
public:
  Authenticate(const string &name, const NodeConfiguration &config) : SyncActionNode(name, config)
  {
  }

  NodeStatus tick() override
  {
    cout << "Authenticate: " << this->name() << endl;
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    return {OutputPort<std::string>("token")};
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
    cout << "GoToDestination: " << this->name() << endl;
    return NodeStatus::SUCCESS;
  }

  static PortsList providedPorts()
  {
    return {InputPort<std::string>("x"), InputPort<std::string>("y")};
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

  factory.registerNodeType<Authenticate>("Authenticate");
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
