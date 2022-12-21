package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MeadowLawnCode extends OpMode {

    private DcMotor left_drive;
    private DcMotor right_drive;
    private CRServo grab_servo;

    @Override
    public void init() {
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        grab_servo = hardwareMap.get(CRServo.class, "grab_servo");


        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    @Override
    public void loop() {

        left_drive.setPower(-gamepad1.left_stick_y);
        right_drive.setPower(gamepad1.right_stick_y);
        grab_servo.setPower(gamepad1.right_trigger - gamepad1.left_trigger);


    }
}
