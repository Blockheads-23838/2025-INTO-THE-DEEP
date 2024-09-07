package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Motor running demo", group="Linear OpMode")
public class MotorRunDemoOpmode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor = null;

    @Override
    public void runOpMode() {

        // --------- CODE TO RUN AFTER PRESSING INIT, BEFORE PRESSING PLAY ------------

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motor = hardwareMap.get(DcMotor.class, "motor");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // !! ----------------- CODE TO RUN AFTER PRESSING PLAY ------------------- !!
        //                         Put your actual code here.

        /*
        motor.setPower(1); // Tells the motor to start going full speed forwards.
                           // The motor will continue moving after the program moves to the next instruction
                           // because this instruction only starts it, and never stops it.

        sleep(2000); // waits 2000 milliseconds (1000 milliseconds = 1 second)
                                // this means the motor will run full speed forwards for 2 seconds

         */
/*
        for (int i = 0; i < 10; i++) {
            motor.setPower(i/10.0);
            sleep(500);
        }
        */

        /*
        for (int i = 0; i < 21; i=i+2) {
            motor.setPower(i/22.0);
            sleep(1000);

        */

        while (true) {
            my_function(gamepad1.left_stick_y);


        }
        /*
        left_speed = gamepad1.left_stick_y - gamepad1.right_stick_x;
        right_speed = gamepad1.left_stick_y + gamepad1.right_stick_x;
         */

        // ----------- PROGRAM ENDS HERE --------------

    }

    private void my_function(double input) {
        telemetry.addData("left stick y:", gamepad1.left_stick_y);
        telemetry.update();
        motor.setPower(input);
    }

    private double add_one(double input) {
        return input + 1;
    }
}
