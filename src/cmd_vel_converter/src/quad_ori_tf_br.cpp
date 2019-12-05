#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<tf/transform_broadcaster.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "quad_orientation_br");

    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform quad_ori_fram;

    tf::TransformListener listener;

    ros::Rate rate(10);

    while (node.ok())
    {
        tf::StampedTransform transform;
        try
        {
            ros::Time current = ros::Time::now();
            listener.waitForTransform("world", "base_footprint", current, ros::Duration(3.0));
            listener.lookupTransform("world", "base_footprint", current, transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        quad_ori_fram.setOrigin(tf::Vector3(0, 0, 0));
        quad_ori_fram.setRotation(transform.getRotation());

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "quad_orientation"));
        
        rate.sleep();
    }
    


    return 0;
}