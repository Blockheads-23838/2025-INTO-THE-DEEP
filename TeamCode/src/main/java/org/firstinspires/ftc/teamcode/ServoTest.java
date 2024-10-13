package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo test", group="Linear OpMode")
public class ServoTest extends LinearOpMode {

    private boolean open = false;
    private Servo servo = null;
    private double position = Constants.claw_closed;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {
            open = gamepad1.a;
           if (open) {
               position = Constants.claw_open;
           }
           else {
               position = Constants.claw_closed;
           }

           servo.setPosition(position);

           telemetry.addData("servo angle (degrees): ", position * 300);
           telemetry.addData("gamepad input: ", gamepad1.a);
           telemetry.addData("servo open: ", open);
           telemetry.update();

           sleep(20);
        }
    }
}


