package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="servo test", group="Linear OpMode")
public class ServoTest extends LinearOpMode {

    private CRServo servo = null;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(CRServo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {

            servo.setPower(-gamepad1.right_stick_y);

            telemetry.addData("servo power set to ", -gamepad2.right_stick_y);
            telemetry.update();

            sleep(20);

        }
    }
}
