//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/boustrophedon_stc.hpp"

using nav2_util::declare_parameter_if_not_declared;

int pattern_dir_ = point;

namespace full_coverage_path_planner
{
  BoustrophedonSTC::BoustrophedonSTC()
  {
  }

  BoustrophedonSTC::~BoustrophedonSTC()
  {
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Destroying plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
  }

  void BoustrophedonSTC::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    if (!initialized_)
    {
      // Get node from parent
      node_ = parent.lock();
      name_ = name;

      // Currently this plugin does not use the costmap, instead request a map from a server
      // This will change in the future
      //costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
      global_frame_ = costmap_ros->getGlobalFrameID();

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),
                  "Configuring plugin %s of type NavfnPlanner", name_.c_str());

      // Create a publisher to visualize the plan
      plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);

      // Define  robot radius (radius) parameter
      double robot_radius_default = 0.5;
      declare_parameter_if_not_declared(node_, name_ + ".robot_radius", rclcpp::ParameterValue(robot_radius_default));
      node_->get_parameter(name_ + ".robot_radius", robot_radius_);
      // Define  tool radius (radius) parameter
      double tool_radius_default = 0.5;
      declare_parameter_if_not_declared(node_, name_ + ".tool_radius", rclcpp::ParameterValue(tool_radius_default));
      node_->get_parameter(name_ + ".tool_radius", tool_radius_);
      initialized_ = true;
    }
  }

  void BoustrophedonSTC::activate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Activating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
  }

  void BoustrophedonSTC::deactivate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Deactivating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
  }

  void BoustrophedonSTC::cleanup()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Cleaning up plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    // TODO(clopez) Add proper cleanup
  }

  nav_msgs::msg::Path BoustrophedonSTC::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::Path global_path;
    BoustrophedonSTC::makePlan(start, goal, global_path.poses);

    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    return global_path;
  }
  std::list<gridNode_t> BoustrophedonSTC::boustrophedon(std::vector<std::vector<bool> > const& grid, std::list<gridNode_t>& init,
                                          std::vector<std::vector<bool> >& visited)
  
  {
    int dx, dy, x2, y2, i, nRows = grid.size(), nCols = grid[0].size();
    // Mountain pattern filling of the open space
    // Copy incoming list to 'end'
    std::list<gridNode_t> pathNodes(init);
    // Set starting pos
    x2 = pathNodes.back().pos.x;
    y2 = pathNodes.back().pos.y; 
    // set initial direction based on space visible from initial pos
  /*/  IF GLOBAL VAR = 0*/
        
    int robot_dir = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, point);
  /*  GLOBAL VAR++*/  
  /*/   if global var ==1 /*/
    //int robot_dir = 
        
    // set dx and dy based on robot_dir
    switch(robot_dir) {
      case east: // 1
        dx = +1;
        dy = 0;
        break;
      case west: // 2
        dx = -1;
        dy = 0;
        break;
      case north: // 3
        dx = 0;
        dy = +1;
        break;
      case south: // 4
        dx = 0;
        dy = -1;
        break;
      default:
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Full Coverage Path Planner: NO INITIAL ROBOT DIRECTION CALCULATED. This is a logic error that must be fixed by editing boustrophedon_stc.cpp. Will travel east for now.");
        robot_dir = east;
        dx = +1;
        dy = 0;
        break;
    }
    bool done = false;
    while (!done)
    {
      // 1. drive straight until not a valid move (hit occupied cell or at end of map)

      bool hitWall = false;
      while(!hitWall) {
        x2 += dx;
        y2 += dy;
        if (!validMove(x2, y2, nCols, nRows, grid, visited))
        {
          hitWall = true;
          x2 = pathNodes.back().pos.x;
          y2 = pathNodes.back().pos.y;
          break;
        }
        if (!hitWall) {
          addNodeToList(x2, y2, pathNodes, visited);
        }
      }
  
      // 2. check left and right after hitting wall, then change direction
      if (robot_dir == north || robot_dir == south)
      {
        // if going north/south, then check if (now if it goes east/west is valid move, if it's not, then deadend)
        if (!validMove(x2 + 1, y2, nCols, nRows, grid, visited)
          && !validMove(x2 - 1, y2, nCols, nRows, grid, visited)) {
          // dead end, exit
          done = true;
          break;
        } else if (!validMove(x2 + 1, y2, nCols, nRows, grid, visited)) {
          // east is occupied, travel towards west
          x2--;
          pattern_dir_ = west;
        } else if (!validMove(x2 - 1, y2, nCols, nRows, grid, visited)) {
          // west is occupied, travel towards east
          x2++;
          pattern_dir_ = east;
        } else {
         
          // both sides are opened. If don't have a prefered turn direction, travel towards most open direction
          if (!(pattern_dir_ == east || pattern_dir_ == west)) {
            if (validMove(x2, y2 + 1, nCols, nRows, grid, visited)) {
              pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, north);        
            } else {
              pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, south);
            }
            RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),"rotation dir with most space successful");
          }
          // Get into this following state-> (blocked or visited. valid move) preferred turn direction ***-> variable pattern direction***=> top if right here***-> pattern direction not East r West***-> ( if no preferred turn direction---> travel to most open)
          if (pattern_dir_ = east) {
              x2++;
          } else if (pattern_dir_ = west) {
              x2--;
          }
        }
  
        // add Node to List
        addNodeToList(x2, y2, pathNodes, visited);
  
        // change direction 180 deg (this is when after hit wall, increment by 1 node, then head backwards... this gets added to path list when the loop goes back up) 
        if (robot_dir == north) {
          robot_dir = south;
          dy = -1;
        } else if (robot_dir == south) {
          robot_dir = north;
          dy = 1;
        }
      }
      else if (robot_dir == east || robot_dir == west)
      {
        if (!validMove(x2, y2 + 1, nCols, nRows, grid, visited)
          && !validMove(x2, y2 - 1, nCols, nRows, grid, visited)) {
          // dead end, exit
          done = true;
          break;
        } else if (!validMove(x2, y2 + 1, nCols, nRows, grid, visited)) {
          // north is occupied, travel towards south
          y2--;
          pattern_dir_ = south;
        } else if (!validMove(x2, y2 - 1, nCols, nRows, grid, visited)) {
          // south is occupied, travel towards north
          y2++;
          pattern_dir_ = north;
        } else {
          // both sides are opened. If don't have a prefered turn direction, travel towards farthest edge
          if (!(pattern_dir_ == north || pattern_dir_ == south)) {
            if (validMove(x2 + 1, y2, nCols, nRows, grid, visited)) {
              pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, east);        
            } else {
              pattern_dir_ = dirWithMostSpace(x2, y2, nCols, nRows, grid, visited, west);
            }
          }
          if (pattern_dir_ = north) {
              y2++;
          } else if (pattern_dir_ = south) {
              y2--;
          }
        }
  
        // add Node to List
        addNodeToList(x2, y2, pathNodes, visited);
        
        // change direction 180 deg
        if (robot_dir == east) {
          robot_dir = west;
          dx = -1;
        } else if (robot_dir == west) {
          robot_dir = east;
          dx = 1;
        }
      }
    }
    // Log
    // printPathNodes(pathNodes);
    return pathNodes;
  }

  std::list<Point_t> BoustrophedonSTC::boustrophedon_stc(std::vector<std::vector<bool>> const &grid,
                                           Point_t &init,
                                           int &multiple_pass_counter,
                                           int &visited_counter)
  {
    int x, y;
    // Initial node is initially set as visited so it does not count
    pattern_dir_ = point;
    multiple_pass_counter = 0;
    visited_counter = 0;

    std::vector<std::vector<bool>> visited;
    visited = grid; // Copy grid matrix
    x = init.x;
    y = init.y;

    // add initial point to pathNodes
    std::list<gridNode_t> pathNodes;
    std::list<Point_t> fullPath;
  
    addNodeToList(x, y, pathNodes, visited);

    std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);  // Retrieve all goalpoints (Cells not visited)--- all open cells
    ///////////
    std::cout << "Goals Left: " << goals.size() << std::endl;
    // how many goals to start with???(all cells not visited?)
    std::list<gridNode_t>::iterator it;

#ifdef DEBUG_PLOT
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Grid before walking is: ");
    printGrid(grid, visited, fullPath);
#endif

    while (goals.size() != 0)
    {
      // Remove all elements from pathNodes list except last element.
      // The last point is the starting point for a new search and A* extends the path from there on
      pathNodes = boustrophedon(grid, pathNodes, visited);
      //visited_counter--; // First point is already counted as visited
      // Plan to closest open Node using A*
      // `goals` is essentially the map, so we use `goals` to determine the distance from the end of a potential path
      //    to the nearest free space
      
#ifdef DEBUG_PLOT
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "A_star_to_open_space is resigning", goals.size());
        printGrid(grid, visited, pathNodes, PatternStart, pathNodes.back());
#endif
      
      for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
      {
        Point_t newPoint = { it->pos.x, it->pos.y };
        //?????is this a pointer or another operation? (Above).
        visited_counter++;
        fullPath.push_back(newPoint);
        //what is fullpath pushback again?-> push all the points in to the path
      }  
      
      goals = map_2_goals(visited, eNodeOpen);  // Retrieve remaining goalpoints
      
      pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
      visited_counter--;  // First point is already counted as visited
      // Plan to closest open Node using A*
      // Pathnodes.back(is starting point)`goals` is essentially the map, so we use `goals` to determine the distance from the end of a potential path
      //    to the nearest free space
      bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
      if (resign)
      {
        RCLCPP_WARN(rclcpp::get_logger("FullCoveragePathPlanner"),"A_star_to_open_space is resigning! This may be due to the open cells outside of the obstacle boundary. Goals Left: %u", goals.size());
        break;
      }
      // Update visited grid
      for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
      {
        if (visited[it->pos.y][it->pos.x])
        {
          multiple_pass_counter++;
        }
        visited[it->pos.y][it->pos.x] = eNodeVisited;
      }
      if (pathNodes.size() > 0)
      {
        multiple_pass_counter--; // First point is already counted as visited
      }

#ifdef DEBUG_PLOT
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Grid with path marked as visited is:");
      gridNode_t BoustrophedonStart = pathNodes.back();
      printGrid(grid, visited, pathNodes, pathNodes.front(), pathNodes.back());
#endif
    }
    return fullPath;
  }                    

  bool BoustrophedonSTC::makePlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal,
                           std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    if (!initialized_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Initialized!");
    }

    //clear the plan, just in case
    plan.clear();
    costmap_ = costmap_ros->getCostmap();
    clock_t begin = clock();
    Point_t startPoint;

    std::vector<std::vector<bool>> grid;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),"grid recieved!!");

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),"Parsing grid to internal representation...");
    if (!parseGrid(costmap_, grid, robot_radius_ * 2, tool_radius_ * 2, start, startPoint))
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "Could not parse retrieved grid");
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),"grid parsed!!");
                
#ifdef DEBUG_PLOT
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Start grid is:");
    std::list<Point_t> printPath;
    printPath.push_back(startPoint);
    printGrid(grid, grid, printPath);
#endif

    std::list<Point_t> goalPoints = boustrophedon_stc(grid,
                                               startPoint,
                                               boustrophedon_cpp_metrics_.multiple_pass_counter,
                                               boustrophedon_cpp_metrics_.visited_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "naive cpp completed!");
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Converting path to plan");

    parsePointlist2Plan(start, goalPoints, plan);
    // Print some metrics:
    boustrophedon_cpp_metrics_.accessible_counter = boustrophedon_cpp_metrics_.visited_counter - boustrophedon_cpp_metrics_.multiple_pass_counter;
    boustrophedon_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * boustrophedon_cpp_metrics_.accessible_counter;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total visited: %d", boustrophedon_cpp_metrics_.visited_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total re-visited: %d", boustrophedon_cpp_metrics_.multiple_pass_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total accessible cells: %d", boustrophedon_cpp_metrics_.accessible_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total accessible area: %f", boustrophedon_cpp_metrics_.total_area_covered);

    // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept
    // (also controlled by planner_frequency parameter in move_base namespace)

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Publishing plan!");
    publishPlan(plan);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan published!");
    RCLCPP_DEBUG(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan published");

    clock_t end = clock();
    double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_secs << "\n";

    return true;
  }
} // namespace full_coverage_path_planner

// register this planner as a nav2_core::GlobalPlanner plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::BoustrophedonSTC, nav2_core::GlobalPlanner)
