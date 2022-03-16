
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
#include "route_planner.h"




// macro for getting the time stamp of a ros message
#define TIME(msg) ( (msg)->header.stamp.toSec() )



RoutePlanner::RoutePlanner()    
{
  using namespace lanelet;
  // viz_timer = nh_.createTimer(ros::Duration(0.1), &MapLoader::viz_pub,this);
  // g_map_pub = nh_.advertise<visualization_msgs::MarkerArray>("lanelet2_map_viz", 1, true);




  
}


void RoutePlanner::setMap(lanelet::LaneletMapPtr map){
  map_ = map;
    ROS_INFO("Route planner loaded");
}    


RoutePlanner::~RoutePlanner()
{}
