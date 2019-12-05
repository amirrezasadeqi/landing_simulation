#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<nav_msgs/Odometry.h>

// writing classes to listen to states of the quadrotor
// and the turtlebot.

class quad_state_listener {
    public:
        // variable to save the state of the quadrotor
        nav_msgs::Odometry quadrotor_state;

        // callback function on quad state receiving
        void quad_state_callback(const nav_msgs::Odometry::ConstPtr& state){

            quadrotor_state.header = state->header;
            quadrotor_state.child_frame_id = state->child_frame_id;
            quadrotor_state.pose = state->pose;
            quadrotor_state.twist = state->twist;

        }

};

class turtlebot_state_listener {
    public:
        // variable to save the turtlebot state
        nav_msgs::Odometry turtlebot_state;
    
        // callback function to save the state of the turtlebot
        void turtlebot_state_callback(const nav_msgs::Odometry::ConstPtr& state){

            turtlebot_state.header = state->header;
            turtlebot_state.child_frame_id = state->child_frame_id;
            turtlebot_state.pose = state->pose;
            turtlebot_state.twist = state->twist;
        }

};

int main(int argc, char** argv){

    ros::init(argc, argv, "wr_quad_vel_publisher");

    ros::NodeHandle node;

    // reading the state of quadrotor
    quad_state_listener quad_state_listener;
    ros::Subscriber quad_state_sub = node.subscribe("/drone/ground_truth/state", 20, &quad_state_listener::quad_state_callback, &quad_state_listener);

    // reading the state of the turtlebot
    turtlebot_state_listener turtlebot_state_listener;
    ros::Subscriber turtlebot_state_sub = node.subscribe("/ugv/odom", 20, &turtlebot_state_listener::turtlebot_state_callback, &turtlebot_state_listener);

    // now let's convert the turtlebot velocity to the world frame
    // to do this we must listen to the tf topic for ugv/base_link
    // to the world. but note that we don't have translation for 
    // velocity:
    // create a transform listener
    tf::TransformListener tf_listener;

    // a variable to save the ugv/base_link to the world transform.
    tf::StampedTransform ugv_world_transform;
    // a transform derived from ugv wrt world for transforming 
    // velocity without translation.
    tf::Transform ugv_world_vel_transform;

    while (node.ok())
    {

        ros::Time current = ros::Time::now();

        // subscribing to the ugv wrt world transform
        try
        {
            tf_listener.waitForTransform("world", "/ugv/base_link", current, ros::Duration(3.0));
            tf_listener.lookupTransform("world", "/ugv/base_link", current, ugv_world_transform);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        // now ugv to world without translation
        ugv_world_vel_transform.
        
        

        // this is for checking the topics and subscribing
        // to them in each loop.
        ros::spinOnce();        
    }
    


    return 0;
}