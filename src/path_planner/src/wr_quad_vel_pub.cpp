// TODO:
// after the td we must say to quadrotor to stop
// if not the commands which we send to /cmd_vel_wr
// are nan.
#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/Point.h>
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

   
    ros::Time start_time;

    // variable to save the quadrotor velocity command to send to ros 
    // network
    geometry_msgs::Point cmd_vel;

    // publisher to publish the cmd_vel
    ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Point>("cmd_vel_wr", 10);

    ros::Rate rate(20);

    int while_count = 0;

    while (node.ok())
    {
        
        ros::Time current = ros::Time::now();
        if(while_count == 1) start_time = current;
        
        double time = current.toSec() - start_time.toSec();

        // an stamped point to save the turtlebot velocity in base_link and world frames
        tf::Stamped<tf::Point> turtle_vel_in, turtle_vel_out;
        turtle_vel_in.frame_id_ = "turtlebot_orientation";
        turtle_vel_in.stamp_ = current;
        turtle_vel_in.setX(turtlebot_state_listener.turtlebot_state.twist.twist.linear.x);
        turtle_vel_in.setY(turtlebot_state_listener.turtlebot_state.twist.twist.linear.y);
        turtle_vel_in.setZ(turtlebot_state_listener.turtlebot_state.twist.twist.linear.z);


        // subscribing to the ugv wrt world transform
        try
        {
            tf_listener.waitForTransform("world", "turtlebot_orientation", current, ros::Duration(3.0));
            tf_listener.transformPoint("world", turtle_vel_in, turtle_vel_out);

        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        // adding some debugging lines of code:
        //std::cout << "the velocity commands are: x = " << turtle_vel_out.getX() << "  y =  " << turtle_vel_out.getY() << "  z =  " << turtle_vel_out.getZ() << std::endl;

        // let's determine the d_0
        //if (turtlebot_state_listener.count == 1){
        if (while_count == 1){
            // some debugging 
            tf::Point quad_init, turtle_init;
            quad_init.setX(quad_state_listener.quadrotor_initial_state.pose.pose.position.x);
            quad_init.setY(quad_state_listener.quadrotor_initial_state.pose.pose.position.y);
            quad_init.setZ(quad_state_listener.quadrotor_initial_state.pose.pose.position.z);
            turtle_init.setX(turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.x);
            turtle_init.setY(turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.y);
            turtle_init.setZ(turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.z);

            // these are some debugging logs
            //std::cout << "initial x position of the quadrotor and turtlebot are respectivly: " << double(quad_init.getX()) << " " << turtle_init.getX() << std::endl;
            //std::cout << "initial y position of the quadrotor and turtlebot are respectivly: " << double(quad_init.getY()) << " " << turtle_init.getY() << std::endl;
            //std::cout << "initial z position of the quadrotor and turtlebot are respectivly: " << double(quad_init.getZ()) << " " << turtle_init.getZ() << std::endl;

            d_0 = sqrt(pow((quad_init.getX() - turtle_init.getX()), 2) + pow((quad_init.getY() - turtle_init.getY()), 2) + pow((quad_init.getZ() - turtle_init.getZ()), 2));
        }

        // debugging for d_0
        //std::cout << "d_0 is : " << d_0 << std::endl;

        // determine d_dot
        double d_dot;
        if(while_count != 0){
            d_dot = (((-2 * d_0 * time) / (kdg * pow(td.toSec(), (2 / kdg)))) * pow((pow(td.toSec(), 2) - pow(time, 2)), ((1 / kdg) - 1)));
        }

        // debugging for the d_dot
        //std::cout << "d_dot is : " << d_dot << std::endl;

        // now determine the quadrotor velocity command and publish it to the ros network
        // note that we use for now the landing target stoped so use the initial state of
        // the turtlebot for x_td.
        // TODO: change the code for moving landing target.

        cmd_vel.x = turtle_vel_out.getX() + (d_dot / d_0) * (quad_state_listener.quadrotor_initial_state.pose.pose.position.x - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.x);
        cmd_vel.y = turtle_vel_out.getY() + (d_dot / d_0) * (quad_state_listener.quadrotor_initial_state.pose.pose.position.y - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.y);
        cmd_vel.z = (d_dot / d_0) * (quad_state_listener.quadrotor_initial_state.pose.pose.position.z - turtlebot_state_listener.turtlebot_initial_state.pose.pose.position.z);

        // now publish the quadrotor velocity command with respaect to the world
        cmd_vel_pub.publish(cmd_vel);

        
        // this is for checking the topics and subscribing
        // to them in each loop.
        ros::spinOnce();    

        while_count++;

        rate.sleep();    
    }
   
    return 0;
}