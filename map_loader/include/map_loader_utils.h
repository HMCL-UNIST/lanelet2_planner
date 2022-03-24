// #pragma once
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>


int ctl_count=100000;

void insertMarkerArray(visualization_msgs::MarkerArray* a1, const visualization_msgs::MarkerArray& a2)
{
  a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
}


void setColor(std_msgs::ColorRGBA* cl, double r, double g, double b, double a)
{
  cl->r = r;
  cl->g = g;
  cl->b = b;
  cl->a = a;
}

lanelet::ConstLanelets laneletLayerConst(lanelet::LaneletMapPtr ll_map)
{
  lanelet::ConstLanelets lanelets;  

  for (auto li = ll_map->laneletLayer.begin(); li != ll_map->laneletLayer.end(); li++)
  {
    lanelets.push_back(*li);
  }

  return lanelets;
}


lanelet::Lanelets laneletLayer(lanelet::LaneletMapPtr ll_map)
{
  lanelet::Lanelets lanelets;  

  for (auto li = ll_map->laneletLayer.begin(); li != ll_map->laneletLayer.end(); li++)
  {
    lanelets.push_back(*li);
  }

  return lanelets;
}

lanelet::Lanelets subtypeLanelets(lanelet::Lanelets lls, const char subtype[])
{
  lanelet::Lanelets subtype_lanelets;

  for (auto li = lls.begin(); li != lls.end(); li++)
  {
    lanelet::Lanelet ll = *li;

    if (ll.hasAttribute(lanelet::AttributeName::Subtype))
    {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype)
      {
        subtype_lanelets.push_back(ll);
      }
    }
  }

  return subtype_lanelets;
}

lanelet::ConstLanelets subtypeLanelets(lanelet::ConstLanelets lls, const char subtype[])
{
  lanelet::ConstLanelets subtype_lanelets;

  for (auto li = lls.begin(); li != lls.end(); li++)
  {
    lanelet::ConstLanelet ll = *li;

    if (ll.hasAttribute(lanelet::AttributeName::Subtype))
    {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype)
      {
        subtype_lanelets.push_back(ll);
      }
    }
  }

  return subtype_lanelets;
}

// lanelet::Lanelets crosswalkLanelets(const lanelet::Lanelets lls)
// {
//   return (subtypeLanelets(lls, lanelet::AttributeValueString::Crosswalk));
// }

lanelet::Lanelets roadLanelets(lanelet::Lanelets lls)
{
  return (subtypeLanelets(lls, lanelet::AttributeValueString::Road));
}

lanelet::ConstLanelets roadLaneletsConst(lanelet::ConstLanelets lls)
{
  return (subtypeLanelets(lls, lanelet::AttributeValueString::Road));
}




std::vector<std::shared_ptr<const lanelet::TrafficLight>> get_trafficLights(lanelet::Lanelets lanelets)
{ 
  std::vector<std::shared_ptr<const lanelet::TrafficLight>> tl_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++)
  {
    lanelet::ConstLanelet ll = *i;
    std::vector<std::shared_ptr<const lanelet::TrafficLight>> ll_tl_re = ll.regulatoryElementsAs<lanelet::TrafficLight>();

    // insert unique tl into array
    for (auto tli = ll_tl_re.begin(); tli != ll_tl_re.end(); tli++)
    {
      std::shared_ptr<const lanelet::TrafficLight> tl_ptr = *tli;
      lanelet::Id id = tl_ptr->id();
      bool unique_id = true;
      for (auto ii = tl_reg_elems.begin(); ii != tl_reg_elems.end(); ii++)
      {
        if (id == (*ii)->id())
        {
          unique_id = false;
          break;
        }
      }
      if (unique_id)
      {
        tl_reg_elems.push_back(tl_ptr);
      }
    }
  }
  return tl_reg_elems;
}

void toGeomMsgPt(const geometry_msgs::Point32& src, geometry_msgs::Point* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x;
  dst->y = src.y;
  dst->z = src.z;
}
void toGeomMsgPt(const Eigen::Vector3d& src, geometry_msgs::Point* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}
void toGeomMsgPt(const lanelet::ConstPoint3d& src, geometry_msgs::Point* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}
void toGeomMsgPt(const lanelet::ConstPoint2d& src, geometry_msgs::Point* dst)
{
  if (dst == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "pointer is null!");
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = 0;
}

geometry_msgs::Point toGeomMsgPt(const geometry_msgs::Point32& src)
{
  geometry_msgs::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::Point toGeomMsgPt(const Eigen::Vector3d& src)
{
  geometry_msgs::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::Point toGeomMsgPt(const lanelet::ConstPoint3d& src)
{
  geometry_msgs::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}
geometry_msgs::Point toGeomMsgPt(const lanelet::ConstPoint2d& src)
{
  geometry_msgs::Point dst;
  toGeomMsgPt(src, &dst);
  return dst;
}



void trafficLight2TriangleMarker(const lanelet::ConstLineString3d ls, visualization_msgs::Marker* marker,
                                                const std::string ns, const std_msgs::ColorRGBA cl,int tl_count)
{
  if (marker == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": marker is null pointer!");
    return;
  }
  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time();
  marker->ns = ns;
  // marker->id = ls.id();
  marker->id = tl_count+20000;  
  marker->type = visualization_msgs::Marker::TRIANGLE_LIST;

  marker->lifetime = ros::Duration();

  marker->pose.position.x = 0.0;  // p.x();
  marker->pose.position.y = 0.0;  // p.y();
  marker->pose.position.z = 0.0;  // p.z();
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  marker->color.r = 1.0f;
  marker->color.g = 1.0f;
  marker->color.b = 1.0f;
  marker->color.a = 1.0f;

  double h = 0.7;
  if (ls.hasAttribute("height"))
  {
    lanelet::Attribute attr = ls.attribute("height");
    h = std::stod(attr.value());
  }

  // construct triangles and add to marker

  // define polygon of traffic light border
  Eigen::Vector3d v[4];
  v[0] << ls.front().x(), ls.front().y(), ls.front().z();
  v[1] << ls.back().x(), ls.back().y(), ls.back().z();
  v[2] << ls.back().x(), ls.back().y(), ls.back().z() + h;
  v[3] << ls.front().x(), ls.front().y(), ls.front().z() + h;

  Eigen::Vector3d c = (v[0] + v[1] + v[2] + v[3]) / 4;
  double scale = 2.0;
  if (scale > 0.0 && scale != 1.0)
  {
    for (int i = 0; i < 4; i++)
    {
      v[i] = (v[i] - c) * scale + c;
    }
  }
  geometry_msgs::Point tri0[3];
  toGeomMsgPt(v[0], &tri0[0]);
  toGeomMsgPt(v[1], &tri0[1]);
  toGeomMsgPt(v[2], &tri0[2]);
  geometry_msgs::Point tri1[3];
  toGeomMsgPt(v[0], &tri1[0]);
  toGeomMsgPt(v[2], &tri1[1]);
  toGeomMsgPt(v[3], &tri1[2]);

  for (int i = 0; i < 3; i++)
  {
    marker->points.push_back(tri0[i]);
    marker->colors.push_back(cl);
  }
  for (int i = 0; i < 3; i++)
  {
    marker->points.push_back(tri1[i]);
    marker->colors.push_back(cl);
  }
}


visualization_msgs::MarkerArray trafficLightsAsTriangleMarkerArray(
    const std::vector<std::shared_ptr<const lanelet::TrafficLight>> tl_reg_elems, const std_msgs::ColorRGBA c)
{
  // convert to to an array of linestrings and publish as marker array using
  // exisitng function

  int tl_count = 0;
  std::vector<lanelet::ConstLineString3d> line_strings;
  visualization_msgs::MarkerArray marker_array;

  for (auto tli = tl_reg_elems.begin(); tli != tl_reg_elems.end(); tli++)
  {
    std::shared_ptr<const lanelet::TrafficLight> tl = *tli;
    lanelet::LineString3d ls;

    auto lights = tl->trafficLights();
    for (auto lsp : lights)
    {
      if (lsp.isLineString())  // traffic ligths can either polygons or
      {                        // linestrings
        lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);

        visualization_msgs::Marker marker;
        trafficLight2TriangleMarker(ls, &marker, "traffic_light_triangle", c,tl_count);
        marker_array.markers.push_back(marker);
        tl_count++;
      }
    }
  }
  
  return (marker_array);
}


void lineString2Marker(const lanelet::ConstLineString3d ls, visualization_msgs::Marker* line_strip,
                                      const std::string frame_id, const std::string ns, const std_msgs::ColorRGBA c,
                                      const float lss, bool is_center_line)
{
  if (line_strip == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": line_strip is null pointer!");
    return;
  }

  line_strip->header.frame_id = frame_id;
  // if(ls.attributeOr("lane_change","no") == "yes"){
  //   std::cout << ls.attributeOr("lane_change","no") << std::endl;
  // }
  // std::cout << ls.attributeOr(lanelet::AttributeName::Subtype,lanelet::AttributeValueString::Dashed) << std::endl;
  
  std::string tmp_s = ls.attributeOr(lanelet::AttributeName::Subtype,lanelet::AttributeValueString::Solid);
  if (tmp_s == "dashed"){    
    std_msgs::ColorRGBA dashed_color;
    setColor(&dashed_color, 1.0, 1.0, 1.0, 0.5);
    line_strip->color = dashed_color;
  }
  else{
    line_strip->color = c;
  }

  line_strip->header.stamp = ros::Time();
  line_strip->ns = ns;
  line_strip->action = visualization_msgs::Marker::ADD;
  ctl_count++;
  line_strip->pose.orientation.w = 1.0;
  
  if(is_center_line){
    line_strip->id = ctl_count+300000;
  }else{
    line_strip->id = ls.id();
    // line_strip->id = ctl_count+500000;
  }

  

  line_strip->type = visualization_msgs::Marker::LINE_STRIP;

  line_strip->scale.x = lss;

  

  // fill out lane line
  for (auto i = ls.begin(); i != ls.end(); i++)
  {
    geometry_msgs::Point p;
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = (*i).z();
    
    line_strip->points.push_back(p);
  }
}


visualization_msgs::MarkerArray laneletsBoundaryAsMarkerArray(lanelet::Lanelets& lanelets,
                                                                             const std_msgs::ColorRGBA c)
{
  double lss = 0.2;  // line string size
  visualization_msgs::MarkerArray marker_array;
  std_msgs::ColorRGBA center_road_c;
  setColor(&center_road_c, 1.0, 0.7, 0.7, 0.3);
  for (auto li = lanelets.begin(); li != lanelets.end(); li++)
  {
    lanelet::Lanelet lll = *li;

    lanelet::LineString3d left_ls = lll.leftBound();
    lanelet::LineString3d right_ls = lll.rightBound();    
    lanelet::ConstLineString3d center_ls = lll.centerline();
    
      

    visualization_msgs::Marker left_line_strip, right_line_strip, center_line_strip;

    lineString2Marker(left_ls, &left_line_strip, "map", "left_lane_bound", c, lss,false);
    lineString2Marker(right_ls, &right_line_strip, "map", "right_lane_bound", c, lss,false);
    marker_array.markers.push_back(left_line_strip);
    marker_array.markers.push_back(right_line_strip);
    // viz_centerline = true;
    // if (viz_centerline)
    // {
    lineString2Marker(center_ls, &center_line_strip, "map", "center_lane_line", center_road_c, lss,true);
    marker_array.markers.push_back(center_line_strip);
    
    // }
  }
  return marker_array;
}


visualization_msgs::MarkerArray lineStringsAsMarkerArray(std::vector<lanelet::LineString3d> line_strings,
                                        const std::string name_space, const std_msgs::ColorRGBA c)
{
  visualization_msgs::MarkerArray ls_marker_array;
  for (auto i = line_strings.begin(); i != line_strings.end(); i++)
  {
    lanelet::ConstLineString3d ls = *i;
    visualization_msgs::Marker ls_marker;

    lineString2Marker(ls, &ls_marker, "map", name_space, c, 0.2,true);

    ls_marker_array.markers.push_back(ls_marker);
  }

  return (ls_marker_array);
}


// return all stop and ref lines from a given lanelet
std::vector<lanelet::LineString3d> getTrafficLightStopLine(lanelet::Lanelet ll)
{
  std::vector<lanelet::LineString3d> stoplines;

  // find stop lines referenced by traffic lights
  std::vector<std::shared_ptr<lanelet::TrafficLight> > traffic_light_reg_elems =
      ll.regulatoryElementsAs<lanelet::TrafficLight>();

  // lanelet has a traffic light elem element
  for (auto reg_elem : traffic_light_reg_elems)
  {
    lanelet::Optional<lanelet::LineString3d> traffic_light_stopline_opt = reg_elem->stopLine();
    if (!!traffic_light_stopline_opt)
    {
      stoplines.push_back(traffic_light_stopline_opt.get());
    }
  }

  return stoplines;
}

// return all stop lines and ref lines from a given set of lanelets
std::vector<lanelet::LineString3d> getTrafficLightStopLines(lanelet::Lanelets lanelets)
{
  std::vector<lanelet::LineString3d> stoplines;

  for (auto lli = lanelets.begin(); lli != lanelets.end(); lli++)
  {
    std::vector<lanelet::LineString3d> ll_stoplines;
    ll_stoplines = getTrafficLightStopLine(*lli);
    stoplines.insert(stoplines.end(), ll_stoplines.begin(), ll_stoplines.end());
  }

  return stoplines;
}



int get_closest_lanelet(const lanelet::ConstLanelets & lanelets, const geometry_msgs::Pose & pose)
{
  double closest_distance = std::numeric_limits<double>::max();
  int closest_index = 0;
  for (int i = 0; i < lanelets.size(); i++) {
    const auto & llt = lanelets.at(i);
    const auto & point2d =
      lanelet::Point2d(lanelet::InvalId, pose.position.x, pose.position.y)
      .basicPoint2d();    
    const double distance = lanelet::geometry::distanceToCenterline2d(llt, point2d);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

