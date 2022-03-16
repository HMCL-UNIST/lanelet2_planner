
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


#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include "map_loader.h"





// macro for getting the time stamp of a ros message
#define TIME(msg) ( (msg)->header.stamp.toSec() )



MapLoader::MapLoader(const ros::NodeHandle& nh) :  
  nh_(nh)  
{
  using namespace lanelet;
  
  viz_timer = nh_.createTimer(ros::Duration(0.1), &MapLoader::viz_pub,this);
  
  g_map_pub = nh_.advertise<visualization_msgs::MarkerArray>("lanelet2_map_viz", 1, true);

  nh_.param<std::string>("map_loader/osm_file_name", osm_file_name, "Town01.osm");
  nh_.getParam("map_loader/osm_file_name", osm_file_name);
  nh_.param<double>("map_loader/map_origin_lat", origin_lat, 0.0);
  nh_.param<double>("map_loader/map_origin_lon", origin_lon, 0.0);
  nh_.param<double>("map_loader/map_origin_att", origin_att, 0.0);
  
  load_map();
  traffic_rules::TrafficRulesPtr trafficRules =
  traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
  routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(*map, *trafficRules);
  // ConstLanelet lanelet = map->laneletLayer.get(1013);
  // routing::LaneletPaths paths = routingGraph->possiblePaths(lanelet, 100, 0, true);
  
  constrcut_viz();


  rp_.setMap(map);
  // nh_.param<int>("num_of_gpsPose_for_icp", num_of_gpsPose_for_icp, 10);
  // nh_.param<bool>("record_transform", record_transform, false);  
  // nh_.param<std::string>("file_name", file_name, "carla_test_v1.csv");

  
 


  
}

MapLoader::~MapLoader()
{}

void MapLoader::constrcut_viz(){
  lanelet::Lanelets all_lanelets = laneletLayer(map);
  lanelet::Lanelets road_lanelets = roadLanelets(all_lanelets);
  
  std::vector<std::shared_ptr<const lanelet::TrafficLight>> tl_reg_elems = get_trafficLights(all_lanelets);

  std_msgs::ColorRGBA cl_road, cl_cross, cl_ll_borders, cl_tl_stoplines, cl_ss_stoplines, cl_trafficlights;
  setColor(&cl_road, 0.2, 0.7, 0.7, 0.3);
  setColor(&cl_cross, 0.2, 0.7, 0.2, 0.3);
  setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 1.0);
  setColor(&cl_tl_stoplines, 1.0, 0.5, 0.0, 0.5);
  setColor(&cl_ss_stoplines, 1.0, 0.0, 0.0, 0.5);
  setColor(&cl_trafficlights, 0.7, 0.7, 0.7, 0.8);

  

  insertMarkerArray(&map_marker_array, laneletsBoundaryAsMarkerArray(
    road_lanelets, cl_ll_borders));
  insertMarkerArray(&map_marker_array, trafficLightsAsTriangleMarkerArray(
    tl_reg_elems, cl_trafficlights));
  
  
  // insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
  //   "road_lanelets", road_lanelets, cl_road));
  // insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
  //   "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  // insertMarkerArray(&map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(
  //   road_lanelets));
  // insertMarkerArray(&map_marker_array, lanelet::visualization::lineStringsAsMarkerArray(
  //   tl_stop_lines, "traffic_light_stop_lines", cl_tl_stoplines, 0.5));
  // insertMarkerArray(&map_marker_array, lanelet::visualization::lineStringsAsMarkerArray(
  //   ss_stop_lines, "stop_sign_stop_lines", cl_ss_stoplines, 0.5));
  // insertMarkerArray(&map_marker_array, lanelet::visualization::autowareTrafficLightsAsMarkerArray(
  //   aw_tl_reg_elems, cl_trafficlights));

  // ROS_INFO("Visualizing lanelet2 map with %lu lanelets, %lu stop lines, and %lu traffic lights",
  //   all_lanelets.size(), tl_stop_lines.size() + ss_stop_lines.size(), aw_tl_reg_elems.size());
 
}
void MapLoader::viz_pub(const ros::TimerEvent& time){  
    g_map_pub.publish(map_marker_array);
}

void MapLoader::load_map(){
  ROS_INFO("map loading");
  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(lanelet::Origin({origin_lat, origin_lon ,origin_att}));
  map = load(osm_file_name, projector,&errors);
  assert(errors.empty()); 
  ROS_INFO("map loaded succesfully");


}


int main (int argc, char** argv)
{
ros::init(argc, argv, "MapLoader");
ros::NodeHandle nh_map_loader;
// ros::NodeHandle nh_route_planner;

MapLoader MapLoader_(nh_map_loader);
// RoutePlanner routePlanner_(nh_route_planner);
ros::spin();
}
