#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void robot_pose_handler(const geometry_msgs::Pose& msg)
{
    // broadcast odom to base_link tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.position.x, msg.position.y, msg.position.z) );
    tf::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "base_link_tf_publisher");
    ros::NodeHandle nh;
    ros::Subscriber robot_pose = nh.subscribe("surena/robot_pose", 1, robot_pose_handler);

    ros::spin();
    return 0;
}