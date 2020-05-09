#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace std;

class GoToGoal{
private:
    ros::Subscriber getPose;
    ros::Publisher pubVel;
    ros::Time oldTime;
    double referencePoints[2];
    double K_P, K_I, K_D;
    double E_A; //accumulated error for integral term in PID
public:
    double* GetReferencePoints();
    double CalculateDistance(double, double, double, double);
    void callback_steerRobot(const turtlesim::Pose& msg);

    GoToGoal(ros::NodeHandle *nh, double x, double y);
};

GoToGoal::GoToGoal(ros::NodeHandle *nh, double x, double y){
    //ROS_INFO("I entered constructor");
    K_P = 3.0;
    K_I = 0.5;
    E_A = 0.0;
    referencePoints[0] = x;
    referencePoints[1] = y;

    pubVel = nh->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    oldTime = ros::Time::now();
    getPose = nh->subscribe("/turtle1/pose", 1000, &GoToGoal::callback_steerRobot, this);
    //ROS_INFO("I left constructor");
}

double* GoToGoal::GetReferencePoints(){
    return referencePoints;
}

double GoToGoal::CalculateDistance(double x1, double y1, double x2, double y2){
    return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

void GoToGoal::callback_steerRobot(const turtlesim::Pose& msg){
    //ROS_INFO("I entered callback");
    double x_c = msg.x;
    double y_c = msg.y;
    double theta_c = msg.theta;
    double *refPoints = GetReferencePoints();
    double theta_d = atan2( (refPoints[1]-y_c), (refPoints[0]-x_c) );
    
    ros::Time currentTime = ros::Time::now();
    ros::Duration diff = currentTime-oldTime;
    oldTime = currentTime;
    double dt = diff.toSec();

    double e = theta_d - theta_c;

    double e_P = e;
    double e_I = E_A + e*dt;

    double omega = K_P * e_P /*+ K_I*e_I*/;

    E_A = e_I;

    //ROS_INFO("%lf", e);

    geometry_msgs::Twist new_vel_data;
    double lin_vel = CalculateDistance(x_c, y_c, refPoints[0], refPoints[1]);

    new_vel_data.angular.z = omega;
    new_vel_data.linear.x = lin_vel;

    pubVel.publish(new_vel_data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_to_goal");
    ros::NodeHandle nh;
    GoToGoal rb1 = GoToGoal(&nh, 5.45, 6.9);
    ros::spin();
}