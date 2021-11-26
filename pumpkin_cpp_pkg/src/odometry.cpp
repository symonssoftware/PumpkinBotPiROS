#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"

class OdometryNode : public rclcpp::Node 
{
public:
    OdometryNode() : Node("odometry") 
    {
        mOdometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 100);
        mJointStatesSubscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 100, 
            std::bind(&OdometryNode::callbackJointStates, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Odometry Node has been started."); 
    }

private:
    void callbackJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        rclcpp::Time currentMessageTime = this->now();

        double dt = (currentMessageTime - mLastMessageTime).nanoseconds();

        double leftVelocityInRadiansPerSecond = msg->velocity.at(0);
        double rightVelocityInRadiansPerSecond = msg->velocity.at(1);

        double leftVelocityInMetersPerSecond = leftVelocityInRadiansPerSecond * 0.07;
        double rightVelocityInMetersPerSecond = rightVelocityInRadiansPerSecond * 0.07;

        double vX = (rightVelocityInMetersPerSecond + leftVelocityInMetersPerSecond) / 2.0;
        double vY = 0.0;
        double vTh = (rightVelocityInMetersPerSecond - leftVelocityInMetersPerSecond) / 0.26;

        double deltaX = (vX * cos(mOdomTh) - vY * sin(mOdomTh)) * dt;
        double deltaY = (vX * sin(mOdomTh) + vY * cos(mOdomTh)) * dt;
        double deltaTh = vTh * dt;

        mOdomX += deltaX;
        mOdomY += deltaY;
        mOdomTh += deltaTh;

        tf2::Quaternion q;
        q.setRPY(0, 0, mOdomTh);

        // Shouldn't need to send this if we're using Robot Localization and EKF
        /*static tf2_ros::TransformBroadcaster transformBroadcaster(this);

            geometry_msgs::msg::TransformStamped odomTransformMsg;
        
            odomTransformMsg.header.stamp = this->now();
            
            odomTransformMsg.header.frame_id = "odom";
            odomTransformMsg.child_frame_id = "base_link";

            odomTransformMsg.transform.translation.x = mOdomX;
            odomTransformMsg.transform.translation.y = mOdomY;
            odomTransformMsg.transform.translation.z = 0.0;

            odomTransformMsg.transform.rotation.x = q.x();
            odomTransformMsg.transform.rotation.y = q.y();
            odomTransformMsg.transform.rotation.z = q.z();
            odomTransformMsg.transform.rotation.w = q.w();

            transformBroadcaster.sendTransform(odomTransformMsg); */

        nav_msgs::msg::Odometry odomMsg;
        odomMsg.header.stamp = currentMessageTime;
        odomMsg.header.frame_id = "odom";
        odomMsg.child_frame_id = "base_link";

        odomMsg.pose.pose.position.x = mOdomX;
        odomMsg.pose.pose.position.y = mOdomY;
        odomMsg.pose.pose.position.z = 0.0;

        odomMsg.pose.pose.orientation.x = q.x();
        odomMsg.pose.pose.orientation.y = q.y();
        odomMsg.pose.pose.orientation.z = q.z();
        odomMsg.pose.pose.orientation.w = q.w();

        odomMsg.twist.twist.linear.x = vX;
        odomMsg.twist.twist.linear.y = vY;
        odomMsg.twist.twist.linear.z = 0.0;
        odomMsg.twist.twist.angular.x = 0.0;
        odomMsg.twist.twist.angular.y = 0.0;
        odomMsg.twist.twist.angular.z = vTh;

        mOdometryPublisher->publish(odomMsg);

        mLastMessageTime = currentMessageTime;
    }

    double mOdomX = 0.0;
    double mOdomY = 0.0;
    double mOdomTh = 0.0;
    rclcpp::Time mLastMessageTime = this->now();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdometryPublisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr mJointStatesSubscriber;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>(); 

    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    rclcpp::spin(node);
    rclcpp::shutdown;
    return 0;
}