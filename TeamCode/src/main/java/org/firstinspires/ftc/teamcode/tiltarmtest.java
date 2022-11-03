package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "tiltarmtest", group = "Iterative Opmode")
public class tiltarmtest extends OpMode{

    private DcMotor tilt_arm = null;

    @Override
    public void init() {
        tilt_arm = hardwareMap.get(DcMotor.class,"tilt_arm");


    }

    @Override
    public void loop() {
        
        tilt_arm.setPower(gamepad2.right_stick_y);

    }
}
