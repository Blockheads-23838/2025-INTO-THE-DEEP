package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo test", group="Linear OpMode")
public class ServoTest extends LinearOpMode {

    private boolean open = false;
    private boolean input = false;
    private Servo servo = null;
    private double position = Constants.claw_closed;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {
            input = gamepad1.a;
            if (input) {
                open = !open;

                if (open) {
                    position = Constants.claw_open;
                }
                else {
                    position = Constants.claw_closed;
                }
            }

           servo.setPosition(position);

           telemetry.addData("servo open: ", open);
           telemetry.addData("input (gamepad a button): ", input);
           telemetry.addData("servo position: ", position);
           telemetry.update();

           sleep(20);
        }
    }
}


