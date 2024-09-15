#include "velocity_commander.hpp"

geometry_msgs::msg::Twist VelocityCommander::generate_velocity_command(double front_dist, double left_dist, double right_dist, double green_percentage, double green_goal_x) 
{
    if (green_percentage >= GOAL_MET_THRESHOLD) 
    {
        linear_speed  = NO_MOVEMENT;
        angular_speed = NO_MOVEMENT;
    } 
    else if (green_percentage >= GOAL_DETECTED_THRESHOLD) 
    {
        handle_green_detection(green_goal_x, linear_speed, angular_speed);
    } 
    else 
    {
        handle_wall_avoidance(front_dist, left_dist, right_dist, linear_speed, angular_speed);
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x  = linear_speed;
    cmd_vel.angular.z = angular_speed;
    return cmd_vel;
}

void VelocityCommander::handle_green_detection(double green_goal_x, double& linear_speed, double& angular_speed)
{
    if (green_goal_x > GOAL_CENTRING_POS) 
    {
        angular_speed = -GOAL_TURN_SPEED; // Turn left
    } 
    else if (green_goal_x < -GOAL_CENTRING_POS) 
    {
        angular_speed = GOAL_TURN_SPEED; // Turn right
    }
    linear_speed = LINEAR_SPEED;
}

void VelocityCommander::handle_wall_avoidance(double front_dist, double left_dist, double right_dist, double& linear_speed, double& angular_speed)
{
    if (left_dist < DIST_THRESHOLD_LEFT && front_dist < DIST_THRESHOLD_FRONT) 
    {
        angular_speed = -TURN_SPEED;
        linear_speed  =  NO_MOVEMENT;
    } 
    else if (left_dist < DIST_THRESHOLD_LEFT) 
    {
        angular_speed = -TURN_SPEED;
    } 
    else if (front_dist < DIST_THRESHOLD_FRONT) 
    {
        angular_speed = TURN_SPEED;
    } 
    else if (right_dist < DIST_THRESHOLD_RIGHT) 
    {
        linear_speed  = LINEAR_SPEED;
        angular_speed = TURN_SPEED_SLOW;
    } 
    else 
    {
        linear_speed  = LINEAR_SPEED;
        angular_speed = TURN_SPEED_SLOW;
    }
}
