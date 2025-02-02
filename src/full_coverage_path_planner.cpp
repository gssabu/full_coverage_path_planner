//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <list>
#include <vector>

#include "full_coverage_path_planner/full_coverage_path_planner.hpp"

/*  *** Note the coordinate system ***
 *  grid[][] is a 2D-vector:
 *  where ix is column-index and x-coordinate in map,
 *  iy is row-index and y-coordinate in map.
 *
 *            Cols  [ix]
 *        _______________________
 *       |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 * Rows  |__|__|__|__|__|__|__|__|
 * [iy]  |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 *y-axis |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *
 *   O   --->> x-axis
 */

// #define DEBUG_PLOT

// Default Constructor
namespace full_coverage_path_planner
{
  FullCoveragePathPlanner::FullCoveragePathPlanner() : initialized_(false)
  {
  }

  void FullCoveragePathPlanner::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    if (!initialized_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    // create a message for the plan
    nav_msgs::msg::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty())
    {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
      gui_path.poses[i] = path[i];
    }

    plan_pub_->publish(gui_path);
  }

  void FullCoveragePathPlanner::parsePointlist2Plan(const geometry_msgs::msg::PoseStamped &start,
                                                    std::list<Point_t> const &goalpoints,
                                                    std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    geometry_msgs::msg::PoseStamped new_goal;
    std::list<Point_t>::const_iterator it, it_next, it_prev;
    int dx_now, dy_now, dx_next, dy_next, move_dir_now = 0, move_dir_next = 0;
    bool do_publish = false;
    float orientation = eDirNone;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Received goalpoints with length: %lu", goalpoints.size());
    if (goalpoints.size() > 1)
    {
      for (it = goalpoints.begin(); it != goalpoints.end(); ++it)
      {
        it_next = it;
        it_next++;
        it_prev = it;
        it_prev--;

        // Check for the direction of movement
        if (it == goalpoints.begin())
        {
          dx_now = it_next->x - it->x;
          dy_now = it_next->y - it->y;
        }
        else
        {
          dx_now = it->x - it_prev->x;
          dy_now = it->y - it_prev->y;
          dx_next = it_next->x - it->x;
          dy_next = it_next->y - it->y;
        }

        // Calculate direction enum: dx + dy*2 will give a unique number for each of the four possible directions because
        // of their signs:
        //  1 +  0*2 =  1
        //  0 +  1*2 =  2
        // -1 +  0*2 = -1
        //  0 + -1*2 = -2
        move_dir_now = dx_now + dy_now * 2;
        move_dir_next = dx_next + dy_next * 2;

        // Check if this points needs to be published (i.e. a change of direction or first or last point in list)
        do_publish = move_dir_next != move_dir_now || it == goalpoints.begin() ||
                     (it != goalpoints.end() && it == --goalpoints.end());

        // Add to vector if required
        if (do_publish)
        {
          new_goal.header.frame_id = "map";
          new_goal.pose.position.x = (it->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
          new_goal.pose.position.y = (it->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;
          // Calculate desired orientation to be in line with movement direction
          switch (move_dir_now)
          {
          case eDirNone:
            // Keep orientation
            break;
          case eDirRight:
            orientation = 0;
            break;
          case eDirUp:
            orientation = M_PI / 2;
            break;
          case eDirLeft:
            orientation = M_PI;
            break;
          case eDirDown:
            orientation = M_PI * 1.5;
            break;
          }
          new_goal.pose.orientation = createQuaternionMsgFromYaw(orientation);
          if (it != goalpoints.begin())
          {
            previous_goal_.pose.orientation = new_goal.pose.orientation;
            // republish previous goal but with new orientation to indicate change of direction
            // useful when the plan is strictly followed with base_link
            plan.push_back(previous_goal_);
          }
          RCLCPP_DEBUG(rclcpp::get_logger("FullCoveragePathPlanner"), "Voila new point: x=%f, y=%f, o=%f,%f,%f,%f", new_goal.pose.position.x, new_goal.pose.position.y,
                       new_goal.pose.orientation.x, new_goal.pose.orientation.y, new_goal.pose.orientation.z,
                       new_goal.pose.orientation.w);
          plan.push_back(new_goal);
          previous_goal_ = new_goal;
        }
      }
    }
    else
    {
      new_goal.header.frame_id = "map";
      new_goal.pose.position.x = (goalpoints.begin()->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
      new_goal.pose.position.y = (goalpoints.begin()->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;
      new_goal.pose.orientation = createQuaternionMsgFromYaw(0);
      plan.push_back(new_goal);
    }
    /* Add poses from current position to start of plan */

    // Compute angle between current pose and first plan point
    double dy = plan.begin()->pose.position.y - start.pose.position.y;
    double dx = plan.begin()->pose.position.x - start.pose.position.x;
    // Arbitrary choice of 100.0*FLT_EPSILON to determine minimum angle precision of 1%
    if (!(fabs(dy) < 100.0 * FLT_EPSILON && fabs(dx) < 100.0 * FLT_EPSILON))
    {
      // Add extra translation waypoint
      double yaw = std::atan2(dy, dx);
      geometry_msgs::msg::Quaternion quat_temp = createQuaternionMsgFromYaw(yaw);
      geometry_msgs::msg::PoseStamped extra_pose;
      extra_pose = *plan.begin();
      extra_pose.pose.orientation = quat_temp;
      plan.insert(plan.begin(), extra_pose);
      extra_pose = start;
      extra_pose.pose.orientation = quat_temp;
      plan.insert(plan.begin(), extra_pose);
    }

    // Insert current pose
    plan.insert(plan.begin(), start);

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan ready containing %lu goals!", plan.size());
  }

  bool FullCoveragePathPlanner::parseCostmap(nav2_costmap_2d::Costmap2D* costmap_grid_,
                                          std::vector<std::vector<bool>> &grid,
                                          float robotRadius,
                                          float toolRadius,
                                          geometry_msgs::msg::PoseStamped const &realStart,
                                          Point_t &scaledStart)
  {
    size_t ix;
    size_t iy;
    size_t nodeRow;
    size_t nodeCol;
    size_t nodeSize = dmax(floor(toolRadius / costmap_grid_->getResolution()), 1);       // Size of node in pixels/units
    //size_t robotNodeSize = dmax(floor(robotRadius / cpp_costmap->getResolution()), 1); // RobotRadius in pixels/units
    size_t nRows = costmap_grid_->getSizeInCellsY();
    size_t nCols = costmap_grid_->getSizeInCellsX();
    unsigned char * costmap_grid_data = costmap_grid_->getCharMap();
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "nRows: %lu nCols: %lu nodeSize: %lu", nRows, nCols, nodeSize);

    if (nRows == 0 || nCols == 0)
    {
      return false;
    }

    // Save map origin and scaling
    //cpp_costmap->mapToWorld(0, 0, grid_origin_.x, grid_origin_.y);
    tile_size_ = nodeSize * costmap_grid_->getResolution(); // Size of a tile in meters
    grid_origin_.x = costmap_grid_->getOriginX();  // x-origin in meters
    grid_origin_.y = costmap_grid_->getOriginY();  // y-origin in meters
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),"costmap resolution: %g", costmap_grid_->getResolution());
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),"tile size: %g", tile_size_);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),"grid origin: (%g, %g)", grid_origin_.x, grid_origin_.y);
    // Scale starting point
        
    scaledStart.x = static_cast<unsigned int>(clamp((realStart.pose.position.x - grid_origin_.x) / tile_size_, 0.0,
                               floor(nCols / tile_size_)));
  
    scaledStart.y = static_cast<unsigned int>(clamp((realStart.pose.position.y - grid_origin_.y) / tile_size_, 0.0,
                               floor(nRows / tile_size_)));
    
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "real start: (%g, %g)", realStart.pose.position.x, realStart.pose.position.y);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "scaled start: (%u, %u)", scaledStart.x, scaledStart.y);
            
    // Scale grid
    for (iy = 0; iy < nRows; iy = iy + nodeSize)
  {
    std::vector<bool> gridRow;
    for (ix = 0; ix < nCols; ix = ix + nodeSize)
    {
      //?????????? where do we specify that above 65 is occupied?? 
      bool nodeOccupied = false;
      for (nodeRow = 0; (nodeRow < nodeSize) && ((iy + nodeRow) < nRows) && (nodeOccupied == false); ++nodeRow)
      {
        //???????????? what does the conditions mean??????   (nodeRow < nodeSize) && ((iy + nodeRow) < nRows)
        for (nodeCol = 0; (nodeCol < nodeSize) && ((ix + nodeCol) < nCols); ++nodeCol)
        {
          double mx = ix + nodeCol;
          double my = iy + nodeRow;
          if (costmap_grid_->getCost(mx, my) > nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          {
            //?????????? how does this if statement become true? 
            nodeOccupied = true;
            // ROS_INFO("(%f, %f) marked occupied", mx, my);
            break;
          }
        }
      }
      gridRow.push_back(nodeOccupied); // what does this push_back mean??
    }
    grid.push_back(gridRow);
  }
  return true;
  }
  bool FullCoveragePathPlanner::parseGrid(nav2_costmap_2d::Costmap2D const * cpp_costmap,
                                          std::vector<std::vector<bool>> &grid,
                                          float robotRadius,
                                          float toolRadius,
                                          geometry_msgs::msg::PoseStamped const &realStart,
                                          Point_t &scaledStart)
  {
    size_t ix;
    size_t iy;
    size_t nodeRow;
    size_t nodeColl;
    size_t nodeSize = dmax(floor(toolRadius / cpp_costmap->getResolution()), 1);       // Size of node in pixels/units
    size_t robotNodeSize = dmax(floor(robotRadius / cpp_costmap->getResolution()), 1); // RobotRadius in pixels/units
    size_t nRows = cpp_costmap->getSizeInCellsY();
    size_t nCols = cpp_costmap->getSizeInCellsX();
    unsigned char * cpp_costmap_data = cpp_costmap->getCharMap();
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "nRows: %lu nCols: %lu nodeSize: %lu", nRows, nCols, nodeSize);

    if (nRows == 0 || nCols == 0)
    {
      return false;
    }

    // Save map origin and scaling
    //cpp_costmap->mapToWorld(0, 0, grid_origin_.x, grid_origin_.y);
    tile_size_ = nodeSize * cpp_costmap->getResolution(); // Size of a tile in meters
    grid_origin_.x = cpp_costmap->getOriginX();  // x-origin in meters
    grid_origin_.y = cpp_costmap->getOriginY();  // y-origin in meters
    // Scale starting point
    scaledStart.x = static_cast<unsigned int>(clamp((realStart.pose.position.x - grid_origin_.x) / tile_size_, 0.0,
                                                    floor(cpp_costmap->getSizeInCellsX() / tile_size_)));
    scaledStart.y = static_cast<unsigned int>(clamp((realStart.pose.position.y - grid_origin_.y) / tile_size_, 0.0,
                                                    floor(cpp_costmap->getSizeInCellsY() / tile_size_)));

    // Scale grid
    for (iy = 0; iy < nRows; iy = iy + nodeSize)
    {
      std::vector<bool> gridRow;
      for (ix = 0; ix < nCols; ix = ix + nodeSize)
      {
        bool nodeOccupied = false;
        for (nodeRow = 0; (nodeRow < robotNodeSize) && ((iy + nodeRow) < nRows) && (nodeOccupied == false); ++nodeRow)
        {
          for (nodeColl = 0; (nodeColl < robotNodeSize) && ((ix + nodeColl) < nCols); ++nodeColl)
          {
            int index_grid = dmax((iy + nodeRow - ceil(static_cast<float>(robotNodeSize - nodeSize) / 2.0)) * nCols + (ix + nodeColl - ceil(static_cast<float>(robotNodeSize - nodeSize) / 2.0)), 0);
            if (cpp_costmap_data[index_grid] > 65)
            {
              nodeOccupied = true;
              break;
            }
          }
        }
        gridRow.push_back(nodeOccupied);
      }
      grid.push_back(gridRow);
    }
    return true;
  }
} // namespace full_coverage_path_planner
