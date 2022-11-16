package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BogieCode extends OpMode {

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightRearDrive;

    private DcMotor tilt_arm;

    private CRServo ExtensionServo;
    private CRServo RotationServo;
    private CRServo GrabServo;
    //private CRServo

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_motor");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_motor");

        tilt_arm = hardwareMap.get(DcMotor.class,"tilt_arm");

        ExtensionServo = hardwareMap.get(CRServo.class, "ExtensionServo");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    @Override
    public void loop() {

        tilt_arm.setPower(gamepad2.left_stick_y);
        ExtensionServo.setPower(gamepad2.right_trigger - gamepad2.left_trigger);


        double drive  = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
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

        leftFrontDrive.setPower(speeds[0]);
        rightFrontDrive.setPower(speeds[1]);
        leftRearDrive.setPower(speeds[2]);
        rightRearDrive.setPower(speeds[3]);

    }
}
