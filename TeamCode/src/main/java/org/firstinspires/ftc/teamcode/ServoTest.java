package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo test", group="Linear OpMode")
public class ServoTest extends LinearOpMode {

    private Boolean open = false;
    private Servo servo = null;
    private Servo servo2 = null;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo 2");

        waitForStart();






        while (opModeIsActive()) {
            // IF toggle button pressed:
                // if open: servo.setPosition(closed)
                // else: servo.setPosition(open)
           if(gamepad1.a == true){
               servo.setPosition(Constants.claw_open);
           }
           else {
               servo.setPosition(Constants.claw_close);
           }




            /* // FIX BELOW

            if (Math.abs(-gamepad1.right_stick_y) > 0) {
                open = ((int) ((int) -gamepad1.right_stick_y + 0.9)) > 0;
            }

            if (open) {
                servo.setDirection(0.9);
                servo2.setPosition(-0.9);
            }

             */ //FIX ABOVE

            telemetry.addData("servo set to ", -gamepad1.right_stick_y);
            //telemetry.addData("open :" , open);
            telemetry.update();

            //sleep(1);
        }
    }
}


