package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="clawtilttest", group="Iterative Opmode")
public class clawtilttest extends OpMode {

    private Servo claw;
    private Servo tilt_claw;
    private Servo odometry_forward;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class,"claw");
        tilt_claw = hardwareMap.get(Servo.class,"tilt_claw");
        odometry_forward = hardwareMap.get(Servo.class, "odometry_forward");

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

        if (gamepad1.dpad_up){
            tilt_claw.setPosition(0.5);
        }
        else if (gamepad1.dpad_down){
            tilt_claw.setPosition(0.6);
        }
        else if (gamepad1.dpad_left){
            tilt_claw.setPosition(.8);
        }
        else if (gamepad1.dpad_right){
            tilt_claw.setPosition(1);
        }

    }
}
