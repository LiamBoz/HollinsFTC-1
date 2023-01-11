package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servopositionaltest", group="Iterative Opmode")
public class servopositionaltesting extends OpMode {
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
            odometry_forward.setPosition(0);
        }
        else if (gamepad1.b){
            odometry_forward.setPosition(0.1);
        }
        else if (gamepad1.a){
            odometry_forward.setPosition(0.25);
        }
        else if (gamepad1.y){
            odometry_forward.setPosition(0.4);
        }

        if (gamepad2.x){
            odometry_forward.setPosition(0.5);
        }
        else if (gamepad2.b){
            odometry_forward.setPosition(0.6);
        }
        else if (gamepad2.a){
            odometry_forward.setPosition(.8);
        }
        else if (gamepad2.y){
            odometry_forward.setPosition(1);
        }

    }
}
