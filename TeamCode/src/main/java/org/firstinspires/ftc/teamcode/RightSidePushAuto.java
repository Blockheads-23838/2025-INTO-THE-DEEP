
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name="Right Side Push Auto", group="Linear OpMode")
public class RightSidePushAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private Servo wrist = null;
    private DcMotorEx pivot = null;
    private Servo clawServo = null;
    private DcMotor slide = null;

    ArrayList<DcMotorEx> driveMotors = new ArrayList<>();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawServo = hardwareMap.get(Servo.class, "claw");
        pivot = hardwareMap.get(DcMotorEx.class, "pivot");
        slide = hardwareMap.get(DcMotor.class, "slide");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        driveMotors.add(leftFrontDrive);
        driveMotors.add(leftBackDrive);
        driveMotors.add(rightFrontDrive);
        driveMotors.add(rightBackDrive);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();




        //fruitloop(give me candy);
        // wrist setmodde
        // wrist run to position
        // wrsist set target position 270





        //goTo(300, 300, 90, 1, true);
        // forward just means that your going north int the direction of the robot. Strafe is side to side. Yaw is turning in degrees.
        // Speed is a number from -1 to 1 x 100 is the percent of speed.
        // Wait to finish means if true then wait for this line of code to finish while false means it can just run other code as well. //
        /* (200, 200, 0, 1,true);
        goTo(0,0,-135, 1, true);
        goTo(200, 0, 1,
                1, true);
        goTo(-500,0,0,1,true);
        goTo(0,0,135,1,true);
        goTo(1100,0,0,1,true);
        goTo(0,0,-180,1,true);
        goTo(1800,0,0,0.5,true);
        goTo(-700,0,0,1,true);
        goTo(0,0,170,1,true);

         */


        /*
        goTo(850, 50,0,1 ,true);
        goTo(-900,0,0,1,true);
        goTo(0,1650,0,1,true);
        goTo(200,0,0,1,true);
        goTo(0,0,-75,1,true);
        goTo(1500,0,0,1,true);
        */

    }
    public void goTo(double forward, double strafe, double yaw, double speed, boolean waitToFinish) {
        /**
         * Moves robot-centrically.  All is in ticks except yaw, which is approximately degrees such that 90 turns the robot very approximately 90 degrees clockwise.
         */
        yaw *= 11;
        double leftFrontPower = speed * (forward + strafe + yaw);
        double rightFrontPower = speed * (forward - strafe - yaw);
        double leftBackPower = speed * (forward - strafe + yaw);
        double rightBackPower = speed * (forward + strafe - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 2000) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
            leftFrontPower *= speed;
            leftBackPower *= speed;
            rightFrontPower *= speed;
            rightBackPower *= speed;
        }

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        leftFrontDrive.setTargetPosition((int) (forward + strafe + yaw));
        leftBackDrive.setTargetPosition((int) (forward - strafe + yaw));
        rightFrontDrive.setTargetPosition((int) (forward - strafe - yaw));
        rightBackDrive.setTargetPosition((int) (forward + strafe - yaw));

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftFrontDrive.setVelocity(leftFrontPower);
        leftBackDrive.setVelocity(leftBackPower);
        rightFrontDrive.setVelocity(rightFrontPower);
        rightBackDrive.setVelocity(rightBackPower);

        if (waitToFinish) while (leftFrontDrive.isBusy()) {
            telemetry.addData("lf power: ", leftFrontPower);
            telemetry.addData("lf tgt position: ", leftFrontDrive.getTargetPosition());
            telemetry.addData("lf position: ", leftFrontDrive.getCurrentPosition());
            telemetry.update();
        }
    }
}

