package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo test", group="Linear OpMode")
public class ServoTest extends LinearOpMode {

    private Boolean open = false;
    private Servo servo = null;
    private Double position = Constants.claw_closed;

    @Override
    public void runOpMode() {

        //servo = hardwareMap.get(Servo.class, "claw");

        waitForStart();






        while (opModeIsActive()) {
            // IF toggle button pressed:
                // if open: servo.setPosition(closed)
                // else: servo.setPosition(open)
           if(gamepad1.a){
               servo.setPosition(Constants.claw_open);
           }
           else {
               servo.setPosition(Constants.claw_closed);
           }

           telemetry.addData("servo angle (degrees): ", position * 300);
           telemetry.update();

            //sleep(1);
        }
    }
}


