package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="James servo test", group="Linear OpMode")
public class JamesServoTest extends LinearOpMode {

    private CRServo servo = null;
    private CRServo servo2 = null;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(CRServo.class, "jamesservo1");
        servo2 = hardwareMap.get(CRServo.class, "jamesservo2");

        waitForStart();


        while (opModeIsActive()) {
            servo.setPower(gamepad1.right_stick_y);
            servo2.setPower(-1 * gamepad1.right_stick_y);


            telemetry.addData("servo set to ", -gamepad1.right_stick_y);
            telemetry.update();

            sleep(20);
        }
    }
}


