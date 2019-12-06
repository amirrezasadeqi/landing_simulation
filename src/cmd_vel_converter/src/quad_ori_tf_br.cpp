#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "quad_orientation_br");

    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform quad_ori_fram;
    tf::Transform quad_xy_frame;
    tf::Transform turtlebot_ori_frame;

    tf::TransformListener listener, turtle_listener;

    ros::Rate rate(20);

    while (node.ok())
    {
        tf::StampedTransform transform, vel_transform;
        try
        {
            ros::Time current = ros::Time::now();
            listener.waitForTransform("world", "base_link", current, ros::Duration(3.0));
            // this is transform between quadrotor and world frame."quad wrt world"
            listener.lookupTransform("world", "base_link", current, transform);

            turtle_listener.waitForTransform("world", "ugv/base_link", current, ros::Duration(3.0));
            turtle_listener.lookupTransform("world", "ugv/base_link", current, vel_transform);

        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        quad_ori_fram.setOrigin(tf::Vector3(0, 0, 0));
        quad_ori_fram.setRotation(transform.getRotation());
        // translational frame of quadrotor in the xy plane
        quad_xy_frame.setOrigin(tf::Vector3(transform.getOrigin().x(), transform.getOrigin().y(), 0));
        quad_xy_frame.setRotation(tf::Quaternion(0, 0, 0, 1));

        // turtlebot orientation frame
        turtlebot_ori_frame.setOrigin(tf::Vector3(0, 0, 0));
        turtlebot_ori_frame.setRotation(vel_transform.getRotation());
        

        br.sendTransform(tf::StampedTransform(quad_ori_fram, ros::Time::now(), "world", "quad_orientation"));
        br.sendTransform(tf::StampedTransform(quad_xy_frame, ros::Time::now(), "world", "quad_translation"));
        br.sendTransform(tf::StampedTransform(turtlebot_ori_frame, ros::Time::now(), "world", "turtlebot_orientation"));

        rate.sleep();
    }

    return 0;
}