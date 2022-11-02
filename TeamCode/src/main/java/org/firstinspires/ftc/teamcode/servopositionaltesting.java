package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servopositionaltest", group="Iterative Opmode")
public class servopositionaltesting extends OpMode {
    private Servo claw;
    private Servo tilt_claw;
    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class,"claw");
        tilt_claw = hardwareMap.get(Servo.class,"tilt_claw");

    }

    @Override
    public void loop() {
        if (gamepad1.x){
            tilt_claw.setPosition(0);
        }
        else if (gamepad1.b){
            tilt_claw.setPosition(0.1);
        }
        else if (gamepad1.a){
            tilt_claw.setPosition(0.25);
        }
        else if (gamepad1.y){
            tilt_claw.setPosition(0.4);
        }

        if (gamepad2.x){
            tilt_claw.setPosition(0.5);
        }
        else if (gamepad2.b){
            tilt_claw.setPosition(0.6);
        }
        else if (gamepad2.a){
            tilt_claw.setPosition(.8);
        }
        else if (gamepad2.y){
            tilt_claw.setPosition(1);
        }

    }
}
