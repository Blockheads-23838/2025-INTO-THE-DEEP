package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public  class Constants {
    // ------------ PIVOT -------------
    public static double pivot_coefficient = 2;
    public static double pivot_kP = 0.001;
    public static double pivot_kF_slide_retracted = 0.3;
    public static double pivot_intake_pose = 80; //encoder counts; lowest the arm should go
    public static double pivot_high_pose = -1600; //encoder counts; highest the arm should go; high basket
    public static double pivot_kF_increase_per_slide_inch_extension = .3;
    public static double pivot_error_multiplier = 0.01;
    //public static double pivot_clicks_per_rotation = 5_281.1;
    //public static double pivot_offset = -15; // this should be in DEGREES.  negative is down.

    //public static double pivot_to_degrees(double pivot_get_current_position_output) {
        //return pivot_get_current_position_output * 360 / pivot_clicks_per_rotation;
    //}

    // ------------ SLIDE -------------

    //public static double slide_max_position = 2500; // todo: change to inches
    public static double slide_kP = 0.001;
    public static double slide_kF = 0.001;
    //public static double slide_clicks_per_inch; // todo: find
    public static double slide_retracted_pose = 5; //encoder counts; minimum slide pose
    public static double slide_max_pose = 2600; //encoder counts; max slide pose

    //public static double slide_to_inches(double slide_get_current_position_output) {
        //return slide_get_current_position_output / slide_clicks_per_inch;
    //}

    //------------- WRIST -------------
    public static double wrist_kP = 0.003;
    public static double wrist_power = 5;
    //public static double wrist_clicks_per_rotation = 360;  // todo: find, this value is because i know it's about that much

    //public static double wrist_to_degrees(double wrist_get_current_position_output) {
        //return wrist_get_current_position_output * 360 / wrist_clicks_per_rotation;
    //}

    // ------------ CLAW --------------
    public static double claw_open = 0.45;
    public static double claw_closed = 0.0;


}
