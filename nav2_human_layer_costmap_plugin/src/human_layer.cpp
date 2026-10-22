/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#include "nav2_human_layer_costmap_plugin/human_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_human_layer_costmap_plugin
{

  HumanLayer::HumanLayer()
      : last_min_x_(-std::numeric_limits<float>::max()),
        last_min_y_(-std::numeric_limits<float>::max()),
        last_max_x_(std::numeric_limits<float>::max()),
        last_max_y_(std::numeric_limits<float>::max())
  {
  }

  // This method is called at the end of plugin initialization.
  // It contains ROS parameter(s) declaration and initialization
  // of need_recalculation_ variable.
  void HumanLayer::onInitialize()
  {
    auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    declareParameter("intimate_radius", rclcpp::ParameterValue(0.45));
    node->get_parameter(name_ + "." + "intimate_radius", intimate_radius_);
    declareParameter("personal_radius", rclcpp::ParameterValue(1.20));
    node->get_parameter(name_ + "." + "personal_radius", personal_radius_);
    declareParameter("social_radius", rclcpp::ParameterValue(3.60));
    node->get_parameter(name_ + "." + "social_radius", social_radius_);
    declareParameter("intimate_cost", rclcpp::ParameterValue(254.0));
    node->get_parameter(name_ + "." + "intimate_cost", intimate_cost_);
    declareParameter("personal_cost", rclcpp::ParameterValue(127.0));
    node->get_parameter(name_ + "." + "personal_cost", personal_cost_);
    declareParameter("social_cost", rclcpp::ParameterValue(0.0));
    node->get_parameter(name_ + "." + "social_cost", social_cost_);
    declareParameter("trajectory_length", rclcpp::ParameterValue(2.0));
    node->get_parameter(name_ + "." + "trajectory_length", trajectory_length_);
    declareParameter("trajectory_width", rclcpp::ParameterValue(0.2));
    node->get_parameter(name_ + "." + "trajectory_width", trajectory_width_);

    humans_subscriber_ = node->create_subscription<Humans>(
        "/humans",
        rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&HumanLayer::humansCallback,
                  this,
                  std::placeholders::_1));

    need_recalculation_ = false;
    current_ = true;
  }

  void HumanLayer::humansCallback(Humans::SharedPtr humansMsg)
  {
    // boost::recursive_mutex::scoped_lock lock(lock_);
    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    // check if humans_ is empty. If it is empty, then we set previous_humans_ to humans_. otherwise we just copy humans_ to previous_humans_
    if (humans_.humans.empty())
    {
      previous_humans_ = *humansMsg;
      humans_ = *humansMsg;
    }
    else
    {
      // Store current time to track updates
      static rclcpp::Time last_update_time = node->now();
      rclcpp::Time current_time = node->now();

      // Update previous_humans_ with current humans_ every 0.1 seconds
      if ((current_time - last_update_time).seconds() >= 0.1) {
        previous_humans_ = humans_;
        last_update_time = current_time;
      }

      // Update specific humans instead of replacing the entire list
      for (const auto &incoming_human : humansMsg->humans) {
        // Find this human in our current list by UUID
        auto it = std::find_if(humans_.humans.begin(), humans_.humans.end(),
              [&incoming_human](const auto &existing_human) {
                return existing_human.uuid == incoming_human.uuid;
              });
        
        if (it != humans_.humans.end()) {
          // Human found, update its data
          *it = incoming_human;
        } else {
          // New human, add to the list
          humans_.humans.push_back(incoming_human);
        }
      }

      // Remove humans that are no longer in the message
      humans_.humans.erase(
        std::remove_if(humans_.humans.begin(), humans_.humans.end(),
              [&humansMsg](const auto &existing_human) {
            return std::none_of(humansMsg->humans.begin(), humansMsg->humans.end(),
                  [&existing_human](const auto &incoming_human) {
                    return incoming_human.uuid == existing_human.uuid;
                  });
              }),
        humans_.humans.end());

    }

    // humans_ = *humansMsg;

    // Info message that shows the list of humans with their ids, stamp and positions.
    // RCLCPP_INFO(node->get_logger(), "Humans received: %lu", humansMsg->humans.size());
    // for (const auto &human : humansMsg->humans)
    // {
    //   RCLCPP_INFO(node->get_logger(), "Human id: %s, stamp: %f, position: (%f, %f)",
    //               human.uuid, human.stamp.sec + human.stamp.nanosec * 1e-9,
    //               human.position.x, human.position.y);
    // }
  }

  // The method is called to ask the plugin: which area of costmap it needs to update.
  // Inside this method window bounds are re-calculated if need_recalculation_ is true
  // and updated independently on its value.
  void HumanLayer::updateBounds(
      double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
      double *min_y, double *max_x, double *max_y)
  {
    if (need_recalculation_)
    {
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      // For some reason when I make these -<double>::max() it does not
      // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
      // -<float>::max() instead.
      *min_x = -std::numeric_limits<float>::max();
      *min_y = -std::numeric_limits<float>::max();
      *max_x = std::numeric_limits<float>::max();
      *max_y = std::numeric_limits<float>::max();
      need_recalculation_ = false;
    }
    else
    {
      double tmp_min_x = last_min_x_;
      double tmp_min_y = last_min_y_;
      double tmp_max_x = last_max_x_;
      double tmp_max_y = last_max_y_;
      last_min_x_ = *min_x;
      last_min_y_ = *min_y;
      last_max_x_ = *max_x;
      last_max_y_ = *max_y;
      *min_x = std::min(tmp_min_x, *min_x);
      *min_y = std::min(tmp_min_y, *min_y);
      *max_x = std::max(tmp_max_x, *max_x);
      *max_y = std::max(tmp_max_y, *max_y);
    }
  }

  // The method is called when footprint was changed.
  // Here it just resets need_recalculation_ variable.
  void
  HumanLayer::onFootprintChanged()
  {
    need_recalculation_ = true;

    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "HumanLayer::onFootprintChanged(): num footprint points: %lu",
                 layered_costmap_->getFootprint().size());
  }

  // The method is called when costmap recalculation is required.
  // It updates the costmap within its window bounds.
  // Inside this method the costmap gradient is generated and is writing directly
  // to the resulting costmap master_grid without any merging with previous layers.
  void
  HumanLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
      int max_i,
      int max_j)
  {
    auto node = node_.lock();

    if (!enabled_)
    {
      return;
    }

    // master_array - is a direct pointer to the resulting master_grid.
    // master_grid - is a resulting costmap combined from all layers.
    // By using this pointer all layers will be overwritten!
    // To work with costmap layer and merge it with other costmap layers,
    // please use costmap_ pointer instead (this is pointer to current
    // costmap layer grid) and then call one of updates methods:
    // - updateWithAddition()
    // - updateWithMax()
    // - updateWithOverwrite()
    // - updateWithTrueOverwrite()
    // In this case using master_array pointer is equal to modifying local costmap_
    // pointer and then calling updateWithTrueOverwrite():
    unsigned char *master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
    // These variables are used to update the costmap only within this window
    // avoiding the updates of whole area.
    //
    // Fixing window coordinates with map size if necessary.
    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    // Iterate through all humans to create safety zones around them
    for (const auto &human : humans_.humans)
    {
      // Skip humans who has not been seen for a long time (compare current stamp with stamp in each human in humans)
      //  Get current time
      rclcpp::Time current_time = node->now();
      // Convert human stamp to rclcpp::Time
      rclcpp::Time human_time = human.stamp;
      // Calculate time difference in seconds
      double time_diff = (current_time - human_time).seconds();

      if (time_diff >= 1.0)
      {
        RCLCPP_DEBUG(node->get_logger(), "Skipping human %s, not seen for too long", human.uuid.c_str());
        continue;
      }

      // Convert world coordinates to map coordinates
      unsigned int mx, my;
      if (!master_grid.worldToMap(human.position.x, human.position.y, mx, my))
      {
        // RCLCPP_WARN(node->get_logger(), "Human position outside of map bounds: (%f, %f)",
        //             human.position.x, human.position.y);
        continue;
      }

      // Calculate the radiuses in cell units
      unsigned int intimate_radius_cells = static_cast<unsigned int>(intimate_radius_ / master_grid.getResolution());
      unsigned int personal_radius_cells = static_cast<unsigned int>(personal_radius_ / master_grid.getResolution());
      unsigned int social_radius_cells = static_cast<unsigned int>(social_radius_ / master_grid.getResolution());

      // Compare humans_ with previous_humans_ to detect movement and predict trajectories
        // Find the matching human in previous data by UUID
        auto prev_it = std::find_if(previous_humans_.humans.begin(), previous_humans_.humans.end(),
                                   [&](const auto &prev_human) {
                                     return prev_human.uuid == human.uuid;
                                   });
        
        // Skip if this human wasn't in the previous frame
        if (prev_it == previous_humans_.humans.end()) {
          continue;
        }
        
        // Calculate time difference between observations
        //rclcpp::Time current_time = current_human.stamp;
        rclcpp::Time prev_time = prev_it->stamp;
        time_diff = std::abs((current_time - prev_time).seconds());
        
        // Calculate position difference
        double dx = human.position.x - prev_it->position.x;
        double dy = human.position.y - prev_it->position.y;
        double distance_diff = std::hypot(dx, dy);
        
        // Only predict trajectory if human moved significantly in a short time
        if (time_diff < 5.0 && distance_diff > 0.1) {
          // Calculate normalized direction vector
          double dir_x = dx / distance_diff;
          double dir_y = dy / distance_diff;
          
          // Set trajectory parameters
          unsigned char trajectory_cost = 254; // Lethal cost
          
          // Calculate endpoint of trajectory
          double end_x = human.position.x + dir_x * trajectory_length_;
          double end_y = human.position.y + dir_y * trajectory_length_;
          
          // Convert trajectory endpoints to map coordinates
          unsigned int mx_start, my_start, mx_end, my_end;
          if (master_grid.worldToMap(human.position.x, human.position.y, mx_start, my_start) &&
              master_grid.worldToMap(end_x, end_y, mx_end, my_end)) {
            
            // Width of trajectory in cells
            int width_cells = std::max(1, static_cast<int>(trajectory_width_ / master_grid.getResolution()));
            
            // Use Bresenham's line algorithm to draw the trajectory
            int dx = mx_end - mx_start;
            int dy = my_end - my_start;
            int steps = std::max(std::abs(dx), std::abs(dy));
            
            if (steps == 0) continue; // Skip if no movement
            
            float x_inc = static_cast<float>(dx) / steps;
            float y_inc = static_cast<float>(dy) / steps;
            
            float x = mx_start;
            float y = my_start;
            
            // Draw the trajectory line with specified width
            for (int i = 0; i <= steps; ++i) {
              int ix = static_cast<int>(x);
              int iy = static_cast<int>(y);
              
              // Apply width by marking cells perpendicular to the line
              for (int wx = -width_cells/2; wx <= width_cells/2; ++wx) {
                for (int wy = -width_cells/2; wy <= width_cells/2; ++wy) {
                  int cell_x = ix + wx;
                  int cell_y = iy + wy;
                  
                  // Check if within bounds and update window
                  if (cell_x >= min_i && cell_x < max_i && 
                      cell_y >= min_j && cell_y < max_j) {
                    unsigned int index = master_grid.getIndex(cell_x, cell_y);
                    // Only update if the new cost is higher
                    if (master_array[index] < trajectory_cost) {
                      master_array[index] = trajectory_cost;
                    }
                  }
                }
              }
              
              x += x_inc;
              y += y_inc;
            }
          }
        }
      

      // Fill in costs for each cell within the radiuses
      for (int i = min_i; i < max_i; i++)
      {
        for (int j = min_j; j < max_j; j++)
        {
          double distance = hypot(i - static_cast<int>(mx), j - static_cast<int>(my));

          // Apply appropriate cost based on distance
          if (distance <= intimate_radius_cells)
          {
            // master_array[master_grid.getIndex(i, j)] = intimate_cost_;
            unsigned char current_cost = master_array[master_grid.getIndex(i, j)];
            if (current_cost < intimate_cost_)
            {
              master_array[master_grid.getIndex(i, j)] = intimate_cost_;
            }
          }
          else if (distance <= personal_radius_cells)
          {
            // master_array[master_grid.getIndex(i, j)] = personal_cost_;

            unsigned char current_cost = master_array[master_grid.getIndex(i, j)];
            if (current_cost < personal_cost_)
            {
              master_array[master_grid.getIndex(i, j)] = personal_cost_;
            }
          }
          else if (distance <= social_radius_cells)
          {
            // Only apply social cost if the cell doesn't already have a higher cost
            unsigned char current_cost = master_array[master_grid.getIndex(i, j)];
            if (current_cost < social_cost_)
            {
              master_array[master_grid.getIndex(i, j)] = social_cost_;
            }
          }
        }
      }
    }
  }
} // namespace nav2_human_layer_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_human_layer_costmap_plugin::HumanLayer, nav2_costmap_2d::Layer)
