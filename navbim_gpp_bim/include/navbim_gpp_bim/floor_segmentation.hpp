// Copyright (c) 2025
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

#ifndef NAVBIM_GPP_BIM__FLOOR_SEGMENTATION_HPP_
#define NAVBIM_GPP_BIM__FLOOR_SEGMENTATION_HPP_

#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "navbim_util/topological_map.hpp"

namespace navbim_gpp_bim
{

/**
 * @brief Information about a floor segment in a path
 */
struct FloorSegment
{
  std::string floor_name;
  std::array<double, 3> start_coords;  // Start point coordinates (x, y, z)
  std::array<double, 3> end_coords;    // End point coordinates (x, y, z)
  size_t start_index;                   // Start index in path (inclusive)
  size_t end_index;                     // End index in path (inclusive)
};

/**
 * @brief Calculate floor segments from a topological path
 * 
 * Segments the path based on floor transitions detected from the topological coarse path.
 * Each segment represents a continuous section of the path on a single floor.
 * 
 * @param coarse_path Sequence of vertices representing the topological path
 * @param graph Topological graph containing node information
 * @param path ROS path message with waypoints
 * @return Vector of floor segments with exact boundary indices
 */
inline std::vector<FloorSegment> calculateFloorSegments(
  const std::vector<navbim_util::Vertex> & coarse_path,
  const navbim_util::TopologicalGraph & graph,
  const nav_msgs::msg::Path & path)
{
  std::vector<FloorSegment> segments;
  
  if (coarse_path.size() < 2 || path.poses.empty()) {
    return segments;
  }
  
  size_t current_segment = 0;
  
  // Iterate through coarse path edges
  for (size_t t = 0; t < coarse_path.size() - 1; ++t) {
    const auto & current_vertex = coarse_path[t];
    const auto & next_vertex = coarse_path[t + 1];
    
    // Get edge between vertices
    auto edge_pair = boost::edge(current_vertex, next_vertex, graph);
    if (!edge_pair.second) {
      continue;  // No edge found, skip
    }
    
    const auto & edge = edge_pair.first;
    const auto & edge_data = graph[edge];
    
    // Check if this is a vertical transition (stair/ramp)
    if (edge_data.subtype.has_value() && 
        (edge_data.subtype.value() == "stair" || edge_data.subtype.value() == "ramp")) {
      // Vertical transition - start new segment after this
      current_segment = segments.size();
    } else {
      // Horizontal transition - part of current floor segment
      if (current_segment == segments.size()) {
        // Start new segment
        std::string floor_name = graph[current_vertex].floor.has_value() ? 
          graph[current_vertex].floor.value().at(0) : "unknown";
        auto start_pos = graph[current_vertex].position;
        auto end_pos = graph[next_vertex].position;
        
        segments.push_back(FloorSegment{
          floor_name,
          {start_pos.x, start_pos.y, start_pos.z},
          {end_pos.x, end_pos.y, end_pos.z},
          0,  // start_index to be filled later
          0   // end_index to be filled later
        });
      } else {
        // Extend current segment by updating end coords
        auto end_pos = graph[next_vertex].position;
        segments.back().end_coords = {end_pos.x, end_pos.y, end_pos.z};
      }
    }
  }
  
  // Helper lambda to find closest waypoint index for given coordinates
  auto find_closest_index = [&path](const std::array<double, 3> & coords) -> size_t {
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & pose = path.poses[i].pose.position;
      double dist = std::sqrt(
        std::pow(pose.x - coords[0], 2) +
        std::pow(pose.y - coords[1], 2) +
        std::pow(pose.z - coords[2], 2));
      
      if (dist < min_dist) {
        min_dist = dist;
        closest_idx = i;
      }
    }
    
    return closest_idx;
  };
  
  // Calculate indices for each segment
  for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
    auto & segment = segments[seg_idx];
    
    // Find closest waypoint to start and end coords
    segment.start_index = find_closest_index(segment.start_coords);
    segment.end_index = find_closest_index(segment.end_coords);
    
    // Special handling for last segment: extend to end of path (inclusive, so last valid index)
    if (seg_idx == segments.size() - 1) {
      segment.end_index = path.poses.size() - 1;
    }
  }
  
  return segments;
}

/**
 * @brief Apply distance-based shift to segment indices for transition zone smoothing
 * 
 * Shifts segment boundaries outward based on distance threshold to include
 * transition zones in smoothing operations.
 * 
 * @param segments Floor segments with exact indices
 * @param path ROS path message
 * @param shift_distance Distance threshold for shifting (in meters)
 * @return New vector with shifted indices
 */
inline std::vector<FloorSegment> applySegmentShift(
  const std::vector<FloorSegment> & segments,
  const nav_msgs::msg::Path & path,
  double shift_distance)
{
  std::vector<FloorSegment> shifted_segments = segments;
  
  for (auto & segment : shifted_segments) {
    size_t original_start = segment.start_index;
    size_t original_end = segment.end_index;
    
    // Shift start index backwards
    if (segment.start_index > 0) {
      for (size_t idx = segment.start_index; ; --idx) {
        double dist = std::sqrt(
          std::pow(path.poses[idx].pose.position.x - path.poses[original_start].pose.position.x, 2) +
          std::pow(path.poses[idx].pose.position.y - path.poses[original_start].pose.position.y, 2));
        
        if (dist <= shift_distance) {
          segment.start_index = (idx > 0) ? idx - 1 : idx;
        } else {
          break;
        }
        
        if (idx == 0) break;
      }
    }
    
    // Shift end index forward
    for (size_t idx = original_end; idx < path.poses.size(); ++idx) {
      double dist = std::sqrt(
        std::pow(path.poses[idx].pose.position.x - path.poses[original_end].pose.position.x, 2) +
        std::pow(path.poses[idx].pose.position.y - path.poses[original_end].pose.position.y, 2));
      
      if (dist <= shift_distance) {
        segment.end_index = (idx + 1 < path.poses.size()) ? idx + 1 : idx;
      } else {
        break;
      }
    }
  }
  
  return shifted_segments;
}

}  // namespace navbim_gpp_bim

#endif  // NAVBIM_GPP_BIM__FLOOR_SEGMENTATION_HPP_
