// Copyright (c) 2025 NavBIM Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAVBIM_UTIL__NODE_REGISTRY_HPP_
#define NAVBIM_UTIL__NODE_REGISTRY_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace navbim_util
{

/**
 * @brief Thread-safe registry for composable nodes to discover each other within the same process
 * 
 * This registry allows composable nodes loaded in the same container to find each other
 * and share pointers directly, avoiding the overhead of service calls or topic communication
 * for frequent interactions.
 * 
 * Usage:
 * - Node registers itself: NodeRegistry::registerNode("my_node", shared_from_this());
 * - Other node looks it up: auto node = NodeRegistry::getNode<MyNodeType>("my_node");
 * - Node unregisters on cleanup: NodeRegistry::unregisterNode("my_node");
 * 
 * This pattern is only useful in composition mode where nodes share the same process.
 * In standalone mode, nodes should use standard ROS communication (topics/services).
 */
class NodeRegistry
{
public:
  /**
   * @brief Register a node in the registry
   * @param name Unique identifier for the node (typically the node name)
   * @param node Shared pointer to the node
   */
  template<typename T>
  static void registerNode(const std::string & name, std::shared_ptr<T> node)
  {
    std::lock_guard<std::mutex> lock(getMutex());
    getRegistry()[name] = std::static_pointer_cast<void>(node);
  }

  /**
   * @brief Look up a node in the registry
   * @param name Unique identifier for the node
   * @return Shared pointer to the node, or nullptr if not found
   */
  template<typename T>
  static std::shared_ptr<T> getNode(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(getMutex());
    auto it = getRegistry().find(name);
    if (it != getRegistry().end()) {
      return std::static_pointer_cast<T>(it->second);
    }
    return nullptr;
  }

  /**
   * @brief Unregister a node from the registry
   * @param name Unique identifier for the node
   */
  static void unregisterNode(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(getMutex());
    getRegistry().erase(name);
  }

  /**
   * @brief Check if a node is registered
   * @param name Unique identifier for the node
   * @return true if the node is registered, false otherwise
   */
  static bool hasNode(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(getMutex());
    return getRegistry().find(name) != getRegistry().end();
  }

private:
  // Use Meyer's singleton pattern to avoid static initialization order issues
  static std::map<std::string, std::shared_ptr<void>> & getRegistry()
  {
    static std::map<std::string, std::shared_ptr<void>> registry;
    return registry;
  }

  static std::mutex & getMutex()
  {
    static std::mutex mutex;
    return mutex;
  }
};

}  // namespace navbim_util

#endif  // NAVBIM_UTIL__NODE_REGISTRY_HPP_
