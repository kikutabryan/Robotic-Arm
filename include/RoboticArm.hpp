#pragma once
#include <Arduino.h>
#include <vector>

typedef std::vector<double> double_vec;

#define PI 3.141592653589

class RoboticArm
{
    RoboticArm(double first_joint_length, double second_joint_length, double gripper_length);

private:
    // ***********************************Variables***********************************
    // Positions of joints in XYZ (mm, mm, mm)
    double_vec first_joint_xyz = double_vec(3);
    double_vec second_joint_xyz = double_vec(3);
    double_vec gripper_xyz = double_vec(3);

    // Positions of joints in XY + rotation (mm, mm, degrees)
    double_vec first_joint_xyr = double_vec(3);
    double_vec second_joint_xyr = double_vec(3);
    double_vec gripper_xyr = double_vec(3);

    // Angles of the joints of the arm with respect to the prior joint (degrees)
    double base_angle;
    double first_joint_angle;
    double second_joint_angle;
    double gripper_joint_angle;

    // Angle of the gripper with respect to the horizontal
    double gripper_command_angle;

    // Lengths of the joints of the arm
    double first_joint_length;
    double second_joint_length;
    double gripper_length;

    // Angle limits of the joints of the arm
    double min_angle_1 = -40;
    double max_angle_1 = 40;
    double min_angle_2 = -40;
    double max_angle_2 = 40;
    double min_angle_3 = -40;
    double max_angle_3 = 40;

    // Initial positions


    // Manual control
    long unsigned last_time_arm = millis();
    long unsigned last_time_base = millis();
    double displacement_arm = 1;
    double displacement_base = 1;

    // ***********************************Functions***********************************
    double degrees_to_rad(double degree_angle);
    double rad_to_degrees(double rad_angle);
    double_vec xyz_to_xyr(double_vec xyz);
    double_vec xyr_to_xyz(double_vec xyr);

    bool in_bounds(double coordinate_x, double coordinate_y);
    void inverse_kinematics(double_vec xyz);
    void manual_xy_rotation(double arm_velocity, double arm_velocity_angle, double angular_velocity, double gripper_angle);

public:
    // ***********************************Variables***********************************


    // ***********************************Functions***********************************
    void change_angle_limits(double min_angle_1, double max_angle_1, double min_angle_2, double max_angle_2);
};