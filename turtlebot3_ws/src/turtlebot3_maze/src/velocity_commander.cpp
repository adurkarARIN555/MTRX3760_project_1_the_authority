// ***************************************
// src/velocity_commander.cpp
//
// Implementation for VelocityCommander class
// ***************************************

#include "velocity_commander.hpp"

geometry_msgs::msg::Twist VelocityCommander::GenerateVelocityCommand(double front_dist, double left_dist, double right_dist, double green_percentage, double green_goal_x) 
{
    if (green_percentage >= GOAL_MET_THRESHOLD) 
    {
        // Stop if the green color percentage exceeds the goal met threshold
        linear_speed  = NO_MOVEMENT;
        angular_speed = NO_MOVEMENT;
    } 
    else if (green_percentage >= GOAL_DETECTED_THRESHOLD) 
    {
        // Handle movement towards the green goal if the green color percentage is above detection threshold
        HandleGreenDetection(green_goal_x, linear_speed, angular_speed);
    } 
    else 
    {
        // Handle obstacle avoidance when no significant green detection
        HandleWallAvoidance(front_dist, left_dist, right_dist, linear_speed, angular_speed);
    }

    // Create and return a twist message with the calculated velocities
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x  = linear_speed;
    cmd_vel.angular.z = angular_speed;
    return cmd_vel;
}

void VelocityCommander::HandleGreenDetection(double green_goal_x, double& linear_speed, double& angular_speed)
{
    if (green_goal_x > GOAL_CENTRING_POS) 
    {
        // Turn left if the green goal is to the right of the center
        angular_speed = -GOAL_TURN_SPEED;
    } 
    else if (green_goal_x < -GOAL_CENTRING_POS) 
    {
        // Turn right if the green goal is to the left of the center
        angular_speed = GOAL_TURN_SPEED;
    }
    // Move forward towards the goal
    linear_speed = LINEAR_SPEED;
}

void VelocityCommander::HandleWallAvoidance(double front_dist, double left_dist, double right_dist, double& linear_speed, double& angular_speed)
{
    if (left_dist < DIST_THRESHOLD_LEFT && front_dist < DIST_THRESHOLD_FRONT) 
    {
        // Turn left if both front and left distances are too short
        angular_speed = -TURN_SPEED;
        linear_speed  =  NO_MOVEMENT;
    } 
    else if (left_dist < DIST_THRESHOLD_LEFT) 
    {
        // Turn left if only the left distance is too short
        angular_speed = -TURN_SPEED;
    } 
    else if (front_dist < DIST_THRESHOLD_FRONT) 
    {
        // Turn right if only the front distance is too short
        angular_speed = TURN_SPEED;
    } 
    else if (right_dist < DIST_THRESHOLD_RIGHT) 
    {
        // Move forward with a slight turn if the right distance is too short
        linear_speed  = LINEAR_SPEED;
        angular_speed = TURN_SPEED_SLOW;
    } 
    else 
    {
        // Move forward with a slight turn otherwise
        linear_speed  = LINEAR_SPEED;
        angular_speed = TURN_SPEED_SLOW;
    }
}
