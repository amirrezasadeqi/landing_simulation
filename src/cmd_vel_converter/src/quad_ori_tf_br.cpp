#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "quad_orientation_br");

    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform quad_ori_fram;
    tf::Transform quad_xy_frame;

    tf::TransformListener listener;

    ros::Rate rate(10);

    while (node.ok())
    {
        tf::StampedTransform transform;
        try
        {
            ros::Time current = ros::Time::now();
            listener.waitForTransform("world", "base_link", current, ros::Duration(3.0));
            // this is transform between quadrotor and world frame."quad wrt world"
            listener.lookupTransform("world", "base_link", current, transform);
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


        br.sendTransform(tf::StampedTransform(quad_ori_fram, ros::Time::now(), "world", "quad_orientation"));
        br.sendTransform(tf::StampedTransform(quad_xy_frame, ros::Time::now(), "world", "quad_translation"));
        
        rate.sleep();
    }
    


    return 0;
}