#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "navbim_multimap_server/multimap_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<navbim_multimap_server::MultimapServer>();
  
  rclcpp::spin(node->get_node_base_interface());
  
  rclcpp::shutdown();
  return 0;
}