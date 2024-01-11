#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class RobotPose
{
private:
    ros::Subscriber robot_pose_sub;
    geometry_msgs::Pose robot_pose_msg;
    tf::TransformBroadcaster br;
    tf::Transform transform_link;
    tf::Transform transform_footprint;
public:
    RobotPose(ros::NodeHandle* nh)
    {
        robot_pose_sub = nh->subscribe("surena/robot_pose", 1, &RobotPose::robot_pose_handler, this);
    }
    void robot_pose_handler(const geometry_msgs::Pose& msg)
    {
        robot_pose_msg.position.x = msg.position.x;
        robot_pose_msg.position.y = msg.position.y;
        robot_pose_msg.position.z = msg.position.z;
        robot_pose_msg.orientation.x = msg.orientation.x;
        robot_pose_msg.orientation.y = msg.orientation.y;
        robot_pose_msg.orientation.z = msg.orientation.z;
        robot_pose_msg.orientation.w = msg.orientation.w;
    }
    void tf_broadcaster()
    {
        transform_footprint.setOrigin( tf::Vector3(robot_pose_msg.position.x, robot_pose_msg.position.y, 0) );
        tf::Quaternion q(robot_pose_msg.orientation.x,
                         robot_pose_msg.orientation.y,
                         robot_pose_msg.orientation.z,
                         robot_pose_msg.orientation.w);
        transform_footprint.setRotation(q);

        transform_link.setOrigin( tf::Vector3(0, 0, robot_pose_msg.position.z) );
        transform_link.setRotation( tf::Quaternion(0, 0, 0, 1) );

        br.sendTransform(tf::StampedTransform(transform_footprint, ros::Time::now(), "odom", "base_footprint"));
        br.sendTransform(tf::StampedTransform(transform_link, ros::Time::now(), "base_footprint", "base_link"));
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "robot_base_tf_publisher");
    ros::NodeHandle nh;
    RobotPose robot_pose(&nh);
    while(ros::ok())
    {
        robot_pose.tf_broadcaster();
        ros::spinOnce();
    }
    return 0;
}