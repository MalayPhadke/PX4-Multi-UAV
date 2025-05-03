#include "rclcpp/rclcpp.hpp"
#include "px4_map_manager/occupancy_map.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px4_map_manager::occMap>(rclcpp::NodeOptions());
  node->occMap::initMap();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
