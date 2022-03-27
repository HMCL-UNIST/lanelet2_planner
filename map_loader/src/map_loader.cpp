
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
  way_pub = nh_.advertise<hmcl_msgs::LaneArray>("/global_traj", 1, true);
  g_map_pub = nh_.advertise<visualization_msgs::MarkerArray>("/lanelet2_map_viz", 1, true);
  g_traj_timer = nh_.createTimer(ros::Duration(0.5), &MapLoader::global_traj_pub,this);    
  
  pose_init = false;
  pose_sub = nh_.subscribe("/current_pose",1,&MapLoader::poseCallback,this);
  goal_sub = nh_.subscribe("move_base_simple/goal", 1, &MapLoader::callbackGetGoalPose, this);

  nh_p_.param<std::string>("osm_file_name", osm_file_name, "Town01.osm");
  nh_p_.getParam("osm_file_name", osm_file_name);
  nh_p_.param<double>("map_origin_lat", origin_lat, 0.0);
  nh_p_.param<double>("map_origin_lon", origin_lon, 0.0);
  nh_p_.param<double>("map_origin_att", origin_att, 0.0);
  nh_p_.param<bool>("visualize_path", visualize_path, true);

  if(visualize_path){
    g_traj_lanelet_viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("/global_traj_lanelets_viz", 1, true);
    g_traj_viz_pub = nh_.advertise<visualization_msgs::MarkerArray>("/global_traj_viz", 1, true);
    viz_timer = nh_.createTimer(ros::Duration(0.1), &MapLoader::viz_pub,this);    
  }
  
  global_traj_available = false;    
  load_map();
  lanelet::traffic_rules::TrafficRulesPtr trafficRules =
  lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  routingGraph = lanelet::routing::RoutingGraph::build(*map, *trafficRules);  
  construct_lanelets_with_viz();
  rp_.setMap(map);
  
}

MapLoader::~MapLoader()
{}

void MapLoader::callbackGetGoalPose(const geometry_msgs::PoseStampedConstPtr &msg){
  cur_goal = msg->pose;
  global_lane_array.lanes.clear();
  if(pose_init && road_lanelets_const.size() > 0){
      int start_closest_lane_idx = get_closest_lanelet(road_lanelets_const,cur_pose);      
      int goal_closest_lane_idx = get_closest_lanelet(road_lanelets_const,cur_goal);
      // lanelet::Optional<lanelet::routing::LaneletPath> route = routingGraph->shortestPath(road_lanelets[start_closest_lane_idx], road_lanelets[goal_closest_lane_idx], 1);
      lanelet::Optional<lanelet::routing::Route> route = routingGraph->getRoute(road_lanelets_const[start_closest_lane_idx], road_lanelets_const[goal_closest_lane_idx], 0);
      lanelet::routing::LaneletPath local_path = route->shortestPath();
      routingGraph->checkValidity();      
      ROS_INFO("shortest path_size = %d", local_path.size());
        if(local_path.empty()){
          ROS_WARN("[MAP_LOADER] = Path is not found");        
        }
        else{
          // Create base trajectory           
          global_lane_array.header.frame_id = "map";
          global_lane_array.header.stamp = ros::Time::now();          
          
          ///////////////////////// Encode lanelets  /////////////////////////////////////////                        
          int init_lane_idx = 0;
          for (int i =0; i < local_path.size() ; ++i ){                
              hmcl_msgs::Lane ll_;          
              ll_.header = global_lane_array.header;
              ll_.lane_id = local_path[i].id();
              double speed_limit = local_path[i].attributeOr("speed_limit",-1.0);                
              ll_.speed_limit = speed_limit;
              
              // check the lane change flag      
              std::vector<lanelet::routing::LaneletRelation> followingrelations_ = route->followingRelations(local_path[i]);
              if( (followingrelations_.size()==0) && (i < local_path.size()-1)){ll_.lane_change_flag = true;}
              else{ll_.lane_change_flag = false;}  
                           
              ///////////////////////// Encode waypoints  /////////////////////////////////////////              
              auto lstring = local_path[i].centerline();   
              double yaw_tmp;               
              int waypoint_idx_init = 0;
              int waypoint_idx_finish = lstring.size();
              // initial lanelet , add current position as the waypoint
              if(i==init_lane_idx){
                waypoint_idx_init = getClosestWaypoint(true,lstring,cur_pose); 
                hmcl_msgs::Waypoint wp_;  
                wp_.lane_id = init_lane_idx;
                wp_.pose.pose.position.x = cur_pose.position.x;
                wp_.pose.pose.position.y = cur_pose.position.y;
                wp_.pose.pose.position.z = cur_pose.position.z;
                wp_.pose.pose.orientation.x = cur_pose.orientation.x;
                wp_.pose.pose.orientation.y = cur_pose.orientation.y;
                wp_.pose.pose.orientation.z = cur_pose.orientation.z;
                wp_.pose.pose.orientation.w = cur_pose.orientation.w;
                ll_.waypoints.push_back(wp_);
                if(ll_.lane_change_flag){init_lane_idx++;}               
              }
              // terminal lanelet , add terminal position as the waypoint
              if(i == local_path.size()-1){
                waypoint_idx_finish = getClosestWaypoint(false,lstring,cur_goal);               
              }

              for (int j = waypoint_idx_init; j < waypoint_idx_finish; j++ ){
                lanelet::ConstPoint3d p1Const = lstring[j]; 
                // ROS_INFO("x = %f, y = %f, z = %f", p1Const.x(),p1Const.y(),p1Const.z());
                hmcl_msgs::Waypoint wp_;                                    
                wp_.lane_id = ll_.lane_id;
                wp_.pose.pose.position.x = p1Const.x();
                wp_.pose.pose.position.y = p1Const.y();
                wp_.pose.pose.position.z = p1Const.z();
                
                
                // point within the lanelet, Last point will have the same yaw as the previous waypoint
                if(j < lstring.size()-1){      
                  yaw_tmp = get_yaw(lstring[j], lstring[j+1]);                                    
                }else{
                  yaw_tmp = get_yaw(lstring[j-1], lstring[j]);
                }

               

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw_tmp);
                q=q.normalize();
                wp_.pose.pose.orientation.x = q[0];
                wp_.pose.pose.orientation.y = q[1];
                wp_.pose.pose.orientation.z = q[2];
                wp_.pose.pose.orientation.w = q[3];

                ll_.waypoints.push_back(wp_);
              }
               // terminal lanelet , add terminal position as the waypoint
              if(i == local_path.size()-1){                
                hmcl_msgs::Waypoint wp_;  
                wp_.lane_id = init_lane_idx;
                wp_.pose.pose.position.x = cur_goal.position.x;
                wp_.pose.pose.position.y = cur_goal.position.y;
                wp_.pose.pose.position.z = cur_goal.position.z;
                wp_.pose.pose.orientation.x = cur_goal.orientation.x;
                wp_.pose.pose.orientation.y = cur_goal.orientation.y;
                wp_.pose.pose.orientation.z = cur_goal.orientation.z;
                wp_.pose.pose.orientation.w = cur_goal.orientation.w;
                ll_.waypoints.push_back(wp_);
              }

              ///////////////////////// Encode trafficlights  /////////////////////////////////////////              
              auto trafficLightRegelems = local_path[i].regulatoryElementsAs<lanelet::TrafficLight>();
              if(trafficLightRegelems.size()>0){
                //////////// Encode (multiple) traffics light in a single lanelet
                for(int i=0; i < trafficLightRegelems.size(); i++){
                  auto tlRegelem = trafficLightRegelems[i];
                  lanelet::ConstLineStringOrPolygon3d thelight = tlRegelem->trafficLights().front();
                  hmcl_msgs::Trafficlight tl_;                  
                  auto thelight_ls = thelight.lineString();     
                  if(thelight_ls){
                    tl_.pose.x = (thelight_ls->front().x()+thelight_ls->back().x())/2.0;
                    tl_.pose.y = (thelight_ls->front().y()+thelight_ls->back().y())/2.0;
                    tl_.pose.z = (thelight_ls->front().z()+thelight_ls->back().z())/2.0;                    
                  }             
                  auto stopline_ = tlRegelem->stopLine();
                  if (stopline_){
                  tl_.valid_stop_line = true;
                  tl_.stop_line.x = (stopline_->front().x()+stopline_->back().x())/2.0;
                  tl_.stop_line.y = (stopline_->front().y()+stopline_->back().y())/2.0;
                  tl_.stop_line.z = (stopline_->front().z()+stopline_->back().z())/2.0;                         
                  }else{
                    tl_.valid_stop_line = false;                      
                    ROS_WARN("Stop line is not defined .... !!!");
                  } 
                  ll_.trafficlights.push_back(tl_);
                }
              }              
              global_lane_array.lanes.push_back(ll_);

                
            }   
          global_traj_available = true;
          // way_pub.publish(global_lane_array);
          
          if(visualize_path){
              // Construct Traj_lanelet_marker 
              traj_lanelet_marker_array.markers.clear();            
              std::vector<lanelet::ConstLanelet> traj_lanelets;
              for(int i=0; i< local_path.size();i++){traj_lanelets.push_back(local_path[i]);}
                  ROS_INFO("trajc size = %d",traj_lanelets.size());
                      std_msgs::ColorRGBA traj_marker_color;
                      setColor(&traj_marker_color, 0.0, 1.0, 0.0, 0.5);                        
                    insertMarkerArray(&traj_lanelet_marker_array, trajectory_draw(
                    traj_lanelets, traj_marker_color));

              // Construct Traj_marker               
              traj_marker_array.markers.clear();
              for(int i=0 ; i< global_lane_array.lanes.size(); i++){              
                for( int j= 0 ; j <global_lane_array.lanes[i].waypoints.size() ; j++){
                      visualization_msgs::Marker marker_tmp;
                      marker_tmp.header = global_lane_array.header;
                      marker_tmp.id = i*1000+j;
                      marker_tmp.ns = "gtraj";
                      marker_tmp.type = visualization_msgs::Marker::ARROW;
                      marker_tmp.action = visualization_msgs::Marker::ADD;                  
                      marker_tmp.pose.position.x = global_lane_array.lanes[i].waypoints[j].pose.pose.position.x;
                      marker_tmp.pose.position.y = global_lane_array.lanes[i].waypoints[j].pose.pose.position.y;
                      marker_tmp.pose.position.z = global_lane_array.lanes[i].waypoints[j].pose.pose.position.z;
                      marker_tmp.pose.orientation.x = global_lane_array.lanes[i].waypoints[j].pose.pose.orientation.x;
                      marker_tmp.pose.orientation.y = global_lane_array.lanes[i].waypoints[j].pose.pose.orientation.y;
                      marker_tmp.pose.orientation.z = global_lane_array.lanes[i].waypoints[j].pose.pose.orientation.z;
                      marker_tmp.pose.orientation.w = global_lane_array.lanes[i].waypoints[j].pose.pose.orientation.w;
                      marker_tmp.color = traj_marker_color;
                      marker_tmp.lifetime = ros::Duration(0.1);
                      marker_tmp.scale.x = 0.7;
                      marker_tmp.scale.y = 0.5;
                      marker_tmp.scale.z = 0.3;                  
                      traj_marker_array.markers.push_back(marker_tmp);
                    }
              }
            
            }
        }
  }
  else{
    ROS_WARN("[MAP_LOADER] = Current pose is not initialized or map is not loaded");
  }       
}

unsigned int MapLoader::getClosestWaypoint(bool is_start, const lanelet::ConstLineString3d &lstring, geometry_msgs::Pose& point_){
  //input is usually the center line 
  int closest_idx=0;  
  if (is_start){
   closest_idx=lstring.size()-1;  
  }
  
  double min_dist_ = 0.0;
  lanelet::ConstPoint3d end_pConst = lstring[lstring.size()-1]; 
  double dist_from_current_to_end= sqrt(pow((end_pConst.x()-point_.position.x),2) + pow((end_pConst.y()-point_.position.y),2));     
  for (int j = 0; j < lstring.size(); j++ ){
    
    lanelet::ConstPoint3d pConst = lstring[j]; 
    
    double dist_= sqrt(pow(pConst.x()-point_.position.x,2) + pow(pConst.y()-point_.position.y,2));     
    double dist_from_tmp_to_end= sqrt(pow(pConst.x()-end_pConst.x(),2) + pow(pConst.y()-end_pConst.y(),2));     
    
    if( (min_dist_ >= dist_)&& (dist_from_tmp_to_end >= dist_from_current_to_end)){
        closest_idx = j;
        min_dist_ =  dist_;
        ROS_INFO("index = %d, dist_from_current_to_end = %f, dist_from_tmp_to_end = %f", closest_idx, dist_from_current_to_end, dist_from_tmp_to_end);
      }  
     
  }
  //  if((dist_from_tmp_to_end >= dist_from_current_to_end) && (closest_idx >= lstring.size()-1)){
  //       closest_idx = closest_idx+1;
  //     }
  return closest_idx;
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

void MapLoader::construct_lanelets_with_viz(){
  lanelet::Lanelets all_lanelets = laneletLayer(map); 
  lanelet::ConstLanelets all_laneletsConst = laneletLayerConst(map);
  road_lanelets = roadLanelets(all_lanelets);  
  road_lanelets_const = roadLaneletsConst(all_laneletsConst);    
  
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

  ROS_INFO("Visualizing lanelet2 map with %lu lanelets, %lu stop lines",
    all_lanelets.size(), tl_stop_lines.size());
 
}



void MapLoader::global_traj_pub(const ros::TimerEvent& time){  
  if(global_traj_available){
    way_pub.publish(global_lane_array);  
  }  
}

void MapLoader::viz_pub(const ros::TimerEvent& time){  
    g_map_pub.publish(map_marker_array);
    g_traj_lanelet_viz_pub.publish(traj_lanelet_marker_array);
    g_traj_viz_pub.publish(traj_marker_array);
    
}

void MapLoader::load_map(){
  ROS_INFO("map loading");
  lanelet::ErrorMessages errors;
  lanelet::projection::UtmProjector projector(lanelet::Origin({origin_lat, origin_lon ,origin_att}));
  map = load(osm_file_name, projector,&errors);
  assert(errors.empty()); 
  ROS_INFO("map loaded succesfully");
}

double MapLoader::get_yaw(const lanelet::ConstPoint3d & _from, const lanelet::ConstPoint3d &_to ){
    double _angle = std::atan2(_to.y() - _from.y(), _to.x() - _from.x());
    
    while(_angle > M_PI){
      _angle = _angle - 2*M_PI;
    }
    while(_angle < M_PI){
      _angle = _angle + 2*M_PI;
    }
  return _angle;
}


int main (int argc, char** argv)
{
ros::init(argc, argv, "MapLoader");
ros::NodeHandle nh_;
ros::NodeHandle nh_private("~");
MapLoader MapLoader_(nh_,nh_private);
ros::spin();
}
