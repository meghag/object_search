#include "set_marker.h"

using namespace std;

visualization_msgs::Marker 
set_marker(std::string frame_id, std::string ns, int id, int shape,
           geometry_msgs::Point position, float scale, float r, float g, float b, float a)
{
	visualization_msgs::Marker marker;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position = position;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = scale;
	marker.scale.y = scale;
	marker.scale.z = scale;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	marker.lifetime = ros::Duration(5.0);

	return marker;
}

visualization_msgs::Marker
set_marker(std::string frame_id, std::string ns, int id, int shape,
           geometry_msgs::Point position, float scale, float r, float g, float b, float a, std::string text)
{
	visualization_msgs::Marker marker;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position = position;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = scale;
	marker.scale.y = scale;
	marker.scale.z = scale;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	marker.text = text;
	marker.lifetime = ros::Duration(0.0);

	return marker;
}

visualization_msgs::Marker
set_marker(std::string frame_id, std::string ns, int id, int shape,
           geometry_msgs::Pose pose, geometry_msgs::Vector3 scale,
           float r, float g, float b, float a)
{
	visualization_msgs::Marker marker;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = ns;
	marker.id = id;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose = pose;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale = scale;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	marker.lifetime = ros::Duration(0.0);

	return marker;
}
