package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PidArm;

@Disabled
@TeleOp(name="LiamCode")
public class LiamCode extends OpMode {

    DcMotor leftmotor;
    DcMotor rightmotor;
    DcMotor armrotate;
    //DcMotor armlift;
    CRServo intake;
    Servo servolift;
    CRServo servospin;
    PidArm armlift;

    double integralSum = 0;
    double kP, kI, kD = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    int[] mecPositions = new int[]{70, 180, 330};


    @Override
    public void init() {
        leftmotor = hardwareMap.get(DcMotor.class,"leftmotor");
        rightmotor = hardwareMap.get(DcMotor.class,"rightmotor");
        armrotate = hardwareMap.get(DcMotor.class,"armrotate");
        //armlift = hardwareMap.get(DcMotor.class,"armlift");
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        try {
            armlift.setup();
        }
        catch(Exception p_exception) {
            armlift = null;
        }

        try {
            intake = hardwareMap.get(CRServo.class, "intake");

        }
        catch(Exception p_exception) {
            intake = null;

        }
        try {
            servolift = hardwareMap.get(Servo.class, "servolift");

        }
        catch(Exception p_exception) {
            servolift = null;

        }
        try {
            servospin = hardwareMap.get(CRServo.class, "servolift");

        }
        catch(Exception p_exception) {
            servospin = null;

        }


    }

    @Override
    public void loop(){
        leftmotor.setPower(-gamepad2.left_stick_y);
        rightmotor.setPower(-gamepad2.right_stick_y);
        armlift.checkPower();
        armrotate.setPower(-gamepad1.right_stick_x);
        if (gamepad1.a)
            intake.setPower(1);
        else if (gamepad1.y)
                intake.setPower(-1);
        else
            intake.setPower(0);
        if (gamepad1.dpad_up)
            armlift.setArmAngle(60);
        else if (gamepad1.dpad_down)
            armlift.setArmAngle(25);
        else
            armlift.setArmAngle(0);



    }

}
