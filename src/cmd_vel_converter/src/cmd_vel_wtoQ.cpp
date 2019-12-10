#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Twist.h>


class wr_com{

    public:
        geometry_msgs::PointStamped wr_vel_command;

        void wr_com_callback(const geometry_msgs::Point::ConstPtr& msg){

            wr_vel_command.header.frame_id = "world";
            //wr_vel_command.header.stamp = ros::Time::now();
            wr_vel_command.point.x = msg->x;
            wr_vel_command.point.y = msg->y;
            wr_vel_command.point.z = msg->z;

        }

};


int main(int argc, char** argv){

    ros::init(argc, argv, "cmd_vel_wtoQ");

    ros::NodeHandle node;

    // TODO:
    // transform the quadrotor velocity command 
    // from the world frame to the quadrotor body
    // frame(actually in the quadrotor orientation
    // frame).

    // now let's subscribe to the /cmd_vel_wr topic
    wr_com wr_com;
    ros::Subscriber cmd_vel_sub = node.subscribe("/cmd_vel_wr", 20, &wr_com::wr_com_callback, &wr_com);

    // publisher for /drone/cmd_vel command
    ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/drone/cmd_vel", 20);
    geometry_msgs::Twist twist_com;

    // Point variable to save the transformed velocity
    geometry_msgs::PointStamped Quad_vel_point;

    // transform listener to convert quadrotor velocity
    // command from world to quadrotor frame.
    tf::TransformListener tf_listener;

    ros::Rate rate(20);

    while (node.ok())
    {
        try
        {
            tf_listener.waitForTransform("quad_orientation", "world", ros::Time(0), ros::Duration(3.0));
            tf_listener.transformPoint("quad_orientation", wr_com.wr_vel_command, Quad_vel_point);
        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        // note that we send the linear velocity part of twist
        // so now let's publish the converted vel command
        twist_com.linear.x = Quad_vel_point.point.x;
        twist_com.linear.y = Quad_vel_point.point.y;
        twist_com.linear.z = Quad_vel_point.point.z;

        cmd_vel_pub.publish(twist_com);

        ros::spinOnce();

        rate.sleep();
    }
    

    return 0;
}