
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


lanelet::ConstLanelets laneletLayer(const lanelet::LaneletMapPtr ll_map)
{
  lanelet::ConstLanelets lanelets;  

  for (auto li = ll_map->laneletLayer.begin(); li != ll_map->laneletLayer.end(); li++)
  {
    lanelets.push_back(*li);
  }

  return lanelets;
}

lanelet::ConstLanelets subtypeLanelets(const lanelet::ConstLanelets lls, const char subtype[])
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

lanelet::ConstLanelets crosswalkLanelets(const lanelet::ConstLanelets lls)
{
  return (subtypeLanelets(lls, lanelet::AttributeValueString::Crosswalk));
}

lanelet::ConstLanelets roadLanelets(const lanelet::ConstLanelets lls)
{
  return (subtypeLanelets(lls, lanelet::AttributeValueString::Road));
}




void lineString2Marker(const lanelet::ConstLineString3d ls, visualization_msgs::Marker* line_strip,
                                      const std::string frame_id, const std::string ns, const std_msgs::ColorRGBA c,
                                      const float lss)
{
  if (line_strip == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": line_strip is null pointer!");
    return;
  }

  line_strip->header.frame_id = frame_id;
  line_strip->header.stamp = ros::Time();
  line_strip->ns = ns;
  line_strip->action = visualization_msgs::Marker::ADD;

  line_strip->pose.orientation.w = 1.0;
  line_strip->id = ls.id();

  line_strip->type = visualization_msgs::Marker::LINE_STRIP;

  line_strip->scale.x = lss;

  line_strip->color = c;

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


visualization_msgs::MarkerArray laneletsBoundaryAsMarkerArray(const lanelet::ConstLanelets& lanelets,
                                                                             const std_msgs::ColorRGBA c,
                                                                             const bool viz_centerline)
{
  double lss = 0.2;  // line string size
  visualization_msgs::MarkerArray marker_array;
  for (auto li = lanelets.begin(); li != lanelets.end(); li++)
  {
    lanelet::ConstLanelet lll = *li;

    lanelet::ConstLineString3d left_ls = lll.leftBound();
    lanelet::ConstLineString3d right_ls = lll.rightBound();
    lanelet::ConstLineString3d center_ls = lll.centerline();

    visualization_msgs::Marker left_line_strip, right_line_strip, center_line_strip;

    lineString2Marker(left_ls, &left_line_strip, "map", "left_lane_bound", c, lss);
    lineString2Marker(right_ls, &right_line_strip, "map", "right_lane_bound", c, lss);
    marker_array.markers.push_back(left_line_strip);
    marker_array.markers.push_back(right_line_strip);
    if (viz_centerline)
    {
      lineString2Marker(center_ls, &center_line_strip, "map", "center_lane_line", c, lss * 0.5);
      marker_array.markers.push_back(center_line_strip);
    }
  }
  return marker_array;
}

