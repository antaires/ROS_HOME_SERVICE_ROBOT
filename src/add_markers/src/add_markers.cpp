#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

//goals
float pickup[3] = {0, -7, 1};
float dropoff[3] = {0, 7, 1};
float threshold[2] = {0.5f, 1.0f};
bool inTransit = false;
bool dropoffComplete = false;
float current[3] = {pickup[0], pickup[1], pickup[3]};

//map correction odom
float odom_pick[3] = {7, 0, 1};
float odom_drop[3] = {-7, 0, 1};

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   float x = msg->pose.pose.position.x;
   float y = msg->pose.pose.position.y;
   float w = msg->pose.pose.orientation.w;

   //before pickup
   if(!inTransit && !dropoffComplete)
   {
      //if within threshold of pickup, signal pickup
      if(fabs(x - odom_pick[0]) <= threshold[0] 
      && fabs(y - odom_pick[1]) <= threshold[0]
      && fabs(w - odom_pick[2]) <= threshold[1])
      {
         inTransit = true;
         ROS_INFO("package picked up!");
      }
   }

   if (inTransit && !dropoffComplete)
   {
      //if within range of dropoff, signal dropoff
      if(fabs(x - odom_drop[0]) <= threshold[0] 
      && fabs(y - odom_drop[1]) <= threshold[0]
      && fabs(w - odom_drop[2]) <= threshold[1])
      {
         dropoffComplete = true;
         inTransit = false;
         ROS_INFO("package delivered!");
      }
   }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);

  while (ros::ok())
  {
    if(dropoffComplete)
    {
       current[0] = dropoff[0];
       current[1] = dropoff[1];
       current[2] = dropoff[2];
    }

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type as CUBE
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    if(inTransit)
        marker.action = visualization_msgs::Marker::DELETE;
    else 
        marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker.pose.position.x = current[0];
    marker.pose.position.y = current[1];
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = current[3];

    // Set the scale of the marker
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color 
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
  
    marker_pub.publish(marker);

    ros::spinOnce();   
    r.sleep();
  }
}
