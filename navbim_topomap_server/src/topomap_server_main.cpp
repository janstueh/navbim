#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "navbim_topomap_server/topomap_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<navbim_topomap_server::TopomapServer>();
  
  rclcpp::spin(node->get_node_base_interface());
  
  rclcpp::shutdown();
  
  return 0;
}
