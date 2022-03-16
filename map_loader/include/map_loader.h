
//   Copyright (c) 2022 Ulsan National Institute of Science and Technology (UNIST)
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.

//   Authour : Hojin Lee, hojinlee@unist.ac.kr

#include <sstream>
#include <string>
#include <list>
#include <iostream>
#include <fstream>
#include <queue>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <GeographicLib/LocalCartesian.hpp>
#include "BlockingQueue.h"
#include <eigen3/Eigen/Geometry>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <vector>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <map_loader_utils.h>
#include <amathutils.hpp>
#include <route_planner.h>

#define PI 3.14159265358979323846264338



class MapLoader 
{
  
private:
ros::NodeHandle nh_;
ros::NodeHandle nh_p_;
ros::Publisher g_map_pub;

ros::Timer viz_timer;
visualization_msgs::MarkerArray map_marker_array;

RoutePlanner rp_;
lanelet::LaneletMapPtr map;
double origin_lat;
double origin_lon;
double origin_att;

std::string osm_file_name;

// transform from local sensor frame to global sensor frame
tf::StampedTransform l_sensor_to_g_sensor;
tf::TransformListener local_transform_listener;

public:
MapLoader(const ros::NodeHandle& nh, const ros::NodeHandle& nh_p);
~MapLoader();

void load_map();
void constrcut_viz();
void viz_pub(const ros::TimerEvent& time);


// void LocalCallback(geometry_msgs::PoseStampedConstPtr local_pose);


};



