package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public  class Constants {
    public static double pivot_kP = 0.001;
    public static double pivot_kF = 0.3;
    public static double slide_max_position = 2000;
    public static double slide_kP = 0.001;
    public static double slide_kF = 0.001;
    public static double pivot_clicks_per_rotation = 5_281.1;
    public static double wrist_kP = 0.003;
    public static double pivot_offset = Math.toRadians(-15); // this should be in RADIANS.  negative is down.
    public static double claw_open = 0.45;
    public static double claw_closed = 0.0;
}
