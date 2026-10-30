#pragma once
#include <string>

namespace fire_observation_demo
{
struct FleetSelection
{
  std::string uav = "uav1";
  double altitude = 5.0;
  bool enabled = false;
  bool edit_route = false;
};

// Both widgets and the goal tool run on Qt's GUI thread. Linking the application
// against the tool library gives them the same selection object, without waiting
// for an asynchronous ROS settings message at the moment of a mouse click.
FleetSelection & fleet_selection();
}
