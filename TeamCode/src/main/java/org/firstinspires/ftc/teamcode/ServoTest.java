package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo test", group="Linear OpMode")
public class ServoTest extends LinearOpMode {

    private Servo servo = null;
    private Servo servo2 = null;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo 2");

        waitForStart();

        while (opModeIsActive()) {

            servo.setPosition(-gamepad1.right_stick_y);
            servo2.setPosition(-gamepad1.right_stick_y);

            telemetry.addData("servo power set to ", -gamepad2.right_stick_y);
            telemetry.update();

            sleep(20);
        }
    }
}
