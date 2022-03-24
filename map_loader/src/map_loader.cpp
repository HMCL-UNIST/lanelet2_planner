
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
#include <ros/ros.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include "map_loader.h"





// macro for getting the time stamp of a ros message
#define TIME(msg) ( (msg)->header.stamp.toSec() )



MapLoader::MapLoader(const ros::NodeHandle& nh,const ros::NodeHandle& nh_p) :  
  nh_(nh), nh_p_(nh_p)  
{
  // using namespace lanelet;
  g_map_pub = nh_.advertise<visualization_msgs::MarkerArray>("lanelet2_map_viz", 1, true);
  viz_timer = nh_.createTimer(ros::Duration(0.05), &MapLoader::viz_pub,this);

  pose_init = false;
  pose_sub = nh_.subscribe("/current_pose",1,&MapLoader::poseCallback,this);
  goal_sub = nh_.subscribe("move_base_simple/goal", 1, &MapLoader::callbackGetGoalPose, this);
 

  nh_p_.param<std::string>("osm_file_name", osm_file_name, "Town01.osm");
  nh_p_.getParam("osm_file_name", osm_file_name);
  nh_p_.param<double>("map_origin_lat", origin_lat, 0.0);
  nh_p_.param<double>("map_origin_lon", origin_lon, 0.0);
  nh_p_.param<double>("map_origin_att", origin_att, 0.0);
  
  load_map();
  lanelet::traffic_rules::TrafficRulesPtr trafficRules =
  lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  routingGraph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);
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

void MapLoader::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg){
  cur_goal = msg->pose;

  if(pose_init && road_lanelets_const.size() > 0){
      int start_closest_lane_idx = get_closest_lanelet(road_lanelets_const,cur_pose);      
      int goal_closest_lane_idx = get_closest_lanelet(road_lanelets_const,cur_goal);
      // lanelet::Optional<lanelet::routing::LaneletPath> route = routingGraph->shortestPath(road_lanelets[start_closest_lane_idx], road_lanelets[goal_closest_lane_idx], 1);
      lanelet::Optional<lanelet::routing::Route> route = routingGraph->getRoute(road_lanelets_const[start_closest_lane_idx], road_lanelets_const[goal_closest_lane_idx], 0);
      lanelet::routing::LaneletPath shortestPath = route->shortestPath();
      routingGraph->checkValidity();
      // shortestPath->begin();
      ROS_INFO("shortest path_s`ize = %d", shortestPath.size());
        if(shortestPath.empty()){
          ROS_WARN("[MAP_LOADER] = Path is not found");        
        }
        else{
          for (int i =0; i < shortestPath.size() ; ++i ){                
                auto & ll = shortestPath[i];
                ROS_INFO("id = %d",ll.id());
              }
        }
  }
  else{
    ROS_WARN("[MAP_LOADER] = Current pose is not initialized or map is not loaded");
  }     
  
}

void MapLoader::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){  
  if(!pose_init){
    pose_init = true;
  }
  cur_pose = msg->pose;
  pose_x = msg->pose.position.x;
  pose_y = msg->pose.position.x;
  pose_x = msg->pose.position.x;

}

void MapLoader::constrcut_viz(){
  lanelet::Lanelets all_lanelets = laneletLayer(map); 
  lanelet::ConstLanelets all_laneletsConst = laneletLayerConst(map);
  road_lanelets = roadLanelets(all_lanelets);  
  road_lanelets_const = roadLaneletsConst(all_laneletsConst);    
  // for(int i=0; road_lanelets.size(); i++){
  //   road_lanelets_const.push_back(road_lanelets[i]);
  // }  
  
  std::vector<std::shared_ptr<const lanelet::TrafficLight>> tl_reg_elems = get_trafficLights(all_lanelets);
  std::vector<lanelet::LineString3d> tl_stop_lines = getTrafficLightStopLines(road_lanelets);
  std_msgs::ColorRGBA cl_road, cl_cross, cl_ll_borders, cl_tl_stoplines, cl_ss_stoplines, cl_trafficlights;
  setColor(&cl_road, 0.2, 0.7, 0.7, 0.3);
  setColor(&cl_cross, 0.2, 0.7, 0.2, 0.3);
  setColor(&cl_ll_borders, 1.0, 1.0, 0.0, 0.3);
  setColor(&cl_tl_stoplines, 1.0, 0.5, 0.0, 0.5);
  setColor(&cl_ss_stoplines, 1.0, 0.0, 0.0, 0.5);
  setColor(&cl_trafficlights, 0.0, 0.0, 1.0, 0.8);

  

  insertMarkerArray(&map_marker_array, laneletsBoundaryAsMarkerArray(
    road_lanelets, cl_ll_borders));
  insertMarkerArray(&map_marker_array, trafficLightsAsTriangleMarkerArray(
    tl_reg_elems, cl_trafficlights));
  
  insertMarkerArray(&map_marker_array, lineStringsAsMarkerArray(
    tl_stop_lines, "traffic_light_stop_lines", cl_tl_stoplines));

  // insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
  //   "road_lanelets", road_lanelets, cl_road));
  // insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
  //   "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
  // insertMarkerArray(&map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(
  //   road_lanelets));
  
  // insertMarkerArray(&map_marker_array, lanelet::visualization::lineStringsAsMarkerArray(
  //   ss_stop_lines, "stop_sign_stop_lines", cl_ss_stoplines, 0.5));
  // insertMarkerArray(&map_marker_array, lanelet::visualization::autowareTrafficLightsAsMarkerArray(
  //   aw_tl_reg_elems, cl_trafficlights));

  ROS_INFO("Visualizing lanelet2 map with %lu lanelets, %lu stop lines",
    all_lanelets.size(), tl_stop_lines.size());
 
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
ros::NodeHandle nh_;
ros::NodeHandle nh_private("~");

MapLoader MapLoader_(nh_,nh_private);
// RoutePlanner routePlanner_(nh_route_planner);
ros::spin();
}
