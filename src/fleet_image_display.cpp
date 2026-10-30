#include <pluginlib/class_list_macros.hpp>
#include <rviz_default_plugins/displays/image/image_display.hpp>

namespace fire_observation_demo
{
// The installed Jazzy ImageTransportDisplay leaves a Connection pointing at a
// destroyed SubscriberFilter after unsubscribe(). A later unsubscribe (including
// base-class destruction) then locks a freed message_filters signal mutex.
// Keep the workaround inside this package; no installed RViz files are edited.
class FleetImageDisplay : public rviz_default_plugins::displays::ImageDisplay
{
public:
  ~FleetImageDisplay() override { unsubscribe(); }

protected:
  void unsubscribe() override
  {
    if (subscription_) { subscription_callback_.disconnect(); }
    subscription_callback_ = message_filters::Connection();
    subscription_.reset();
  }
};
}

PLUGINLIB_EXPORT_CLASS(fire_observation_demo::FleetImageDisplay, rviz_common::Display)
