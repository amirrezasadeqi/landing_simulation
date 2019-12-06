#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Vector3.h>
#include<math.h>


// parameters of landing algorithm
float kdg, desired_time;

// writing classes to listen to states of the quadrotor
// and the turtlebot.

class quad_state_listener {
    public:
        // variable to save the state of the quadrotor. we can use the 
        // quadrotor_initial_state for determining the d_0.
        nav_msgs::Odometry quadrotor_state, quadrotor_initial_state;
        int count = 0;

        // callback function on quad state receiving
        void quad_state_callback(const nav_msgs::Odometry::ConstPtr& state){

            quadrotor_state.header = state->header;
            quadrotor_state.child_frame_id = state->child_frame_id;
            quadrotor_state.pose = state->pose;
            quadrotor_state.twist = state->twist;

            if(count == 0){
                quadrotor_initial_state = quadrotor_state;
            }

            count = count + 1;
        }

};

class turtlebot_state_listener {
    public:
        // variable to save the turtlebot state
        // we can use the turtlebot_initial_state for determine d_0
        // without transforming to other frames.
        nav_msgs::Odometry turtlebot_state, turtlebot_initial_state;
        int count = 0;
    
        // callback function to save the state of the turtlebot
        void turtlebot_state_callback(const nav_msgs::Odometry::ConstPtr& state){

            turtlebot_state.header = state->header;
            turtlebot_state.child_frame_id = state->child_frame_id;
            turtlebot_state.pose = state->pose;
            turtlebot_state.twist = state->twist;

            if(count == 0){
                turtlebot_initial_state = turtlebot_state;
            }

            count = count + 1;
        }

};

int main(int argc, char** argv){

    ros::init(argc, argv, "wr_quad_vel_publisher");

    ros::NodeHandle node;

    // getting the command line inputs
    kdg = atof(argv[1]);
    desired_time = atof(argv[2]);
    double d_0;

    ros::Time td(desired_time);

    // reading the state of quadrotor
    quad_state_listener quad_state_listener;
    ros::Subscriber quad_state_sub = node.subscribe("/drone/ground_truth/state", 20, &quad_state_listener::quad_state_callback, &quad_state_listener);

    // reading the state of the turtlebot
    turtlebot_state_listener turtlebot_state_listener;
    ros::Subscriber turtlebot_state_sub = node.subscribe("/ugv/odom", 20, &turtlebot_state_listener::turtlebot_state_callback, &turtlebot_state_listener);

    // now let's convert the turtlebot linear velocity to the world frame
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
    
    ros::Time start_time = ros::Time::now();

    // variable to save the quadrotor velocity command to send to ros 
    // network
    geometry_msgs::Vector3 cmd_vel;

    // publisher to publish the cmd_vel
    ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Vector3>("cmd_vel_wr", 10);

    ros::Rate rate(20);

    while (node.ok())
    {
        
        ros::Time current = ros::Time::now();
        
        double time = current.toSec() - start_time.toSec();

        // an stamped point to save the turtlebot velocity in base_link and world frames
        tf::Stamped<tf::Vector3> turtle_vel_in, turtle_vel_out;
        turtle_vel_in.frame_id_ = "turtlebot_orientation";
        turtle_vel_in.stamp_ = current;
        turtle_vel_in.x = turtlebot_state_listener.turtlebot_state.twist.twist.linear.x;
        turtle_vel_in.y = turtlebot_state_listener.turtlebot_state.twist.twist.linear.y;
        turtle_vel_in.z = turtlebot_state_listener.turtlebot_state.twist.twist.linear.z;


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
        ugv_world_vel_transform.setOrigin(tf::Vector3(0, 0, 0));
        ugv_world_vel_transform.setRotation(ugv_world_transform.getRotation());
        

        tf::Transformer transformer;
        // we use turtle_vel_out for vm in the path plannin algorithm
        transformer.transformPoint("world", turtle_vel_in, turtle_vel_out);

        // let's determine the d_0
        if (turtlebot_state_listener.count == 1){
            d_0 = sqrt(pow((quad_state_listener.quadrotor_initial_state.pose.pose.position.x - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.x), 2) + pow((quad_state_listener.quadrotor_initial_state.pose.pose.position.y - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.y), 2) + pow((quad_state_listener.quadrotor_initial_state.pose.pose.position.z - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.z), 2));
        }

        // determine d_dot
        double d_dot = (((-2 * d_0 * time) / (kdg * pow(td.toSec(), (2 / kdg)))) * pow((pow(td.toSec(), 2) - pow(time, 2)), ((1 / kdg) - 1)));

        // now determine the quadrotor velocity command and publish it to the ros network
        // note that we use for now the landing target stoped so use the initial state of
        // the turtlebot for x_td.
        // TODO: change the code for moving landing target.

        cmd_vel.x = turtle_vel_out.x + (d_dot / d_0) * (quad_state_listener.quadrotor_initial_state.pose.pose.position.x - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.x);
        cmd_vel.y = turtle_vel_out.y + (d_dot / d_0) * (quad_state_listener.quadrotor_initial_state.pose.pose.position.y - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.y);
        cmd_vel.z = (d_dot / d_0) * (quad_state_listener.quadrotor_initial_state.pose.pose.position.z - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.z);

        // now publish the quadrotor velocity command with respaect to the world
        cmd_vel_pub.publish(cmd_vel);

        
        // this is for checking the topics and subscribing
        // to them in each loop.
        ros::spinOnce();    

        rate.sleep();    
    }
    


    return 0;
}