#include "RoboticArm.hpp"
#include <Arduino.h>
#include <math.h>

/**
 * @brief Construct a new Robotic Arm:: Robotic Arm object
 *
 * @param first_joint_length
 * @param second_joint_length
 * @param gripper_length
 */
RoboticArm::RoboticArm(double first_joint_length, double second_joint_length, double gripper_length)
{
    this->first_joint_length = first_joint_length;
    this->second_joint_length = second_joint_length;
    this->gripper_length = gripper_length;
}

/**
 * @brief Convert degrees to radians
 *
 * @param degree_angle
 * @return double
 */
double RoboticArm::degrees_to_rad(double degree_angle)
{
    double rad_angle = degree_angle * (PI / 180);
    return rad_angle;
}

/**
 * @brief Converts radians to degrees
 * 
 * @param rad_angle 
 * @return double 
 */
double RoboticArm::rad_to_degrees(double rad_angle)
{
    double degree_angle = rad_angle * (180 / PI);
    return degree_angle;
}

/**
 * @brief Converts xyz coordinates to xy and rotation of base
 * 
 * @param xyz 
 * @return double_vec 
 */
double_vec RoboticArm::xyz_to_xyr(double_vec xyz)
{
    double_vec xyr(3);

    xyr[0] = sqrt(pow(xyz[0], 2) + pow(xyz[1], 2));
    xyr[1] = xyz[2];
    xyr[2] = rad_to_degrees(atan2(xyz[1], xyz [0]));

    return xyr;
}

/**
 * @brief Converts xy and rotation of base to xyz
 * 
 * @param xyr 
 * @return double_vec 
 */
double_vec RoboticArm::xyr_to_xyz(double_vec xyr)
{
    double_vec xyz(3);

    xyz[0] = xyr[0] * cos(degrees_to_rad(xyr[2]));
    xyz[1] = xyr[0] * sin(degrees_to_rad(xyr[2]));
    xyz[2] = xyr[1];

    return xyz;
}

/**
 * @brief Check if the coordinates are within the boundaries of the robotic arm
 *
 * @param coordinate_x
 * @param coordinate_y
 * @return true
 * @return false
 */
bool RoboticArm::in_bounds(double coordinate_x, double coordinate_y)
{
    double current_reach = sqrt(pow(coordinate_x, 2) + pow(coordinate_y, 2)); // Current reach of the arm (radius)

    // Check if within the max reach of the arm
    double max_reach = first_joint_length + second_joint_length;
    if (current_reach > max_reach)
        return false;

    // Check if outside the min reach of the arm
    double x_2 = second_joint_length * cos(degrees_to_rad(min_angle_2)); // Minimums segment two can retract to
    double y_2 = second_joint_length * sin(degrees_to_rad(min_angle_2));
    double min_x = first_joint_length + x_2; // Minimum positions arm can reach
    double min_y = y_2;
    double min_reach = sqrt(pow(min_x, 2) + pow(min_y, 2)); // Minimum radius of the arm
    if (current_reach < min_reach)
        return false;

    // Check if below minimum angle
    double current_angle = atan2(coordinate_y, coordinate_x);
    if (current_angle < min_angle_1)
    {
        // Check if not in overreach of arm
        double overreach_x = first_joint_length * cos(min_angle_1); // Offsets of the overreach circle
        double overreach_y = first_joint_length * sin(min_angle_1);
        double overreach_radius = sqrt(pow((coordinate_x - overreach_x), 2) + pow((coordinate_y - overreach_y), 2)); // Radius of the current position
        if (overreach_radius > second_joint_length)
            return false;
    }

    // Check if above maximum angle
    if (current_angle > max_angle_1)
        return false;

    // Check if not in underreach of arm
    double underreach_x = first_joint_length * cos(max_angle_1); // Offsets of the underreach circle
    double underreach_y = first_joint_length * sin(max_angle_1);
    double underreach_radius = sqrt(pow((coordinate_x - underreach_x), 2) + pow((coordinate_y - underreach_y), 2)); // Radius of the current position
    if (underreach_radius < second_joint_length)
        return false;

    // Return true if positon is within the bounds
    return true;
}

/**
 * @brief Calculates the angles (degrees) of the arm for a given position (XYZ)
 * 
 * @param xyz 
 */
void RoboticArm::inverse_kinematics(double_vec xyz)
{
    double second_joint_inside = (pow(xyz[0], 2) + pow(xyz[1], 2) - pow(first_joint_length, 2) - pow(second_joint_length, 2)) / (2 * first_joint_length * second_joint_length);

    // Exceptions for the robotic arm second joint
    if (second_inside > 1)
        return;
    else if (second_inside < 1)
        return;

    // Angle of the second joint of the robotic arm
    second_joint_angle = rad_to_degrees(acos(second_joint_inside));

    double first_joint_inside_1;
    double first_joint_inside_2;

    // Exceptions for the robotic arm first joint part 1
    if (xyz[0] == 0 && xyz[1] > 0)
        first_joint_inside_1 = PI / 2;
    else if (xyz[0] == 0 && xyz[1] < 0)
        first_joint_inside_1 = -PI / 2;
    else
        first_joint_inside_1 = xyz[1] / xyz[0];

    double first_joint_inside_2_top = second_joint_length * sin(degrees_to_rad(second_joint_angle));
    double first_joint_inside_2_bottom = first_joint_length + second_joint_length * cos(degrees_to_rad(second_joint_angle));

    // Exceptions for the robotic arm first joint part 2
    if (first_joint_inside_2_bottom == 0 && first_joint_inside_2_top > 0)
        first_joint_inside_2 = PI / 2;
    else if (first_joint_inside_2_bottom == 0 && first_joint_inside_2_top)
        first_joint_inside_2 = -PI / 2;
    else
        first_joint_inside_2 = first_joint_inside_2_top / first_joint_inside_2_bottom;

    // Angle of the first joint of the robotic arm
    first_joint_angle = rad_to_degrees(atan(first_joint_inside_1) - atan(first_joint_inside_2));
}

/**
 * @brief Manual control of the robotic arm in the XY plane with base rotation
 * 
 * @param arm_velocity 
 * @param arm_velocity_angle 
 * @param base_velocity 
 * @param gripper_angle 
 */
void RoboticArm::manual_xy_rotation(double arm_velocity, double arm_velocity_angle, double base_velocity, double gripper_angle)
{
    bool update_coordinates = false;

    // Convert xyz to xyr
    second_joint_xyr = xyz_to_xyr(second_joint_xyz);

    // Manual control of arm movement
    double delta_time_arm = displacement_arm / arm_velocity;
    if (millis() - last_time_arm > delta_time_arm * 1000)
    {
        // Calculate the displacement in the x and y directions
        double displacement_x = displacement_arm * cos(degrees_to_rad(arm_velocity_angle));
        double displacement_y = displacement_arm * sin(degrees_to_rad(arm_velocity_angle));
        
        // Check if the updated coordinates are within the boundaries of the arm, if not, check if only one coordinate is when updated
        if (in_bounds(second_joint_xyr[0] + displacement_x, second_joint_xyr[1] + displacement_y))
        {
            second_joint_xyr[0] += displacement_x;
            second_joint_xyr[1] += displacement_y;
        }
        else if (in_bounds(second_joint_xyr[0] + displacement_x, second_joint_xyr[1]))
            second_joint_xyr[0] += displacement_x;
        else if (in_bounds(second_joint_xyr[0], second_joint_xyr[1] + displacement_y))
            second_joint_xyr[1] += displacement_y;

        update_coordinates = true;
    }

    // Manual control of the rotation base
    double delta_time_base = displacement_base / base_velocity;
    if (millis() - last_time_base > delta_time_base * 1000)
    {
        // Update the base angle
        second_joint_xyr[2] += displacement_base;
        
        // Check if base angle is in range [-180, 180]
        if (second_joint_xyr[2] < -180)
            second_joint_xyr[2] += 360;
        else if (second_joint_xyr[2] > 180)
            second_joint_xyr[2] -= 360;

        update_coordinates = true;
    }

    // Check if coordinates were updated, and update if true
    if (update_coordinates == true)
    {
        second_joint_xyz = xyr_to_xyz(second_joint_xyr);
    }

    // Update the position of the gripper
    static double last_gripper_command_angle = gripper_command_angle;
    if (gripper_command_angle != last_gripper_command_angle || update_coordinates == true)
    {
        gripper_xyr[0] = second_joint_xyr[0] + gripper_length * cos(degrees_to_rad(gripper_command_angle));
        gripper_xyr[1] = second_joint_xyr[1] + gripper_length * sin(degrees_to_rad(gripper_command_angle));
        gripper_xyr[2] = second_joint_xyr[2];
        gripper_xyz = xyz_to_xyr(gripper_xyr);
        last_gripper_command_angle = gripper_command_angle;
    }
}

/**
 * @brief Modify the angle limits of the servo motors of the robotic arm
 *
 * @param min_angle_1
 * @param max_angle_1
 * @param min_angle_2
 * @param max_angle_2
 */
void RoboticArm::change_angle_limits(double min_angle_1, double max_angle_1, double min_angle_2, double max_angle_2)
{
    this->min_angle_1 = min_angle_1;
    this->max_angle_1 = max_angle_1;
    this->min_angle_2 = min_angle_2;
    this->max_angle_2 = max_angle_2;
}