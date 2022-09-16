package org.hollins.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mecanum Drive Example", group="Iterative Opmode")
public class MecanumDrive extends OpMode {

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor lift_arm    = null;
    private DcMotor rotate_arm  = null;
    private DcMotor intake      = null;
    private DcMotor spin        = null;
    private Servo   lift_servo  = null;
    float spintime = 0;


    @Override
    public void init() {

        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        lift_arm     = hardwareMap.get(DcMotor.class,"lift_arm");
        rotate_arm   = hardwareMap.get(DcMotor.class,"rotate_arm");
        intake       = hardwareMap.get(DcMotor.class,"intake");
        spin         = hardwareMap.get(DcMotor.class,"spin");
        lift_servo   = hardwareMap.get(Servo.class,"lift_servo");


        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        rotate_arm.setDirection(DcMotorSimple.Direction.REVERSE);

    }



    @Override
    public void loop() {

        lift_arm.setPower(-gamepad2.right_stick_y*0.3 + 0.175);
        rotate_arm.setPower(gamepad2.left_stick_x);
        intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger*0.5 );

        if (gamepad1.right_bumper)
            spin.setPower(0.425);
        else if (gamepad1.left_bumper)
            spin.setPower(-0.425);
        else
            spin.setPower(0.0);

        if (gamepad1.a)
            lift_servo.setPosition(0.79);

        if (gamepad1.y)
            lift_servo.setPosition(0.375);




        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }
}