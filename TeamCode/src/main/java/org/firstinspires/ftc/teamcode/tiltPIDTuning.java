package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TurretMotor;

@Config
@TeleOp(name="tiltPIDTuning", group="Iterative Opmode")
public class tiltPIDTuning extends OpMode{
    //public TurretMotor tilt_arm;
    //public TurretMotor rotate_arm;
    public TurretMotor tilt_arm;

    public DcMotor /*tilt, rotate,*/ tilt;

    public Servo tilt_claw;

    public static double tiltP = 0.002, tiltI = 0.00, tiltD = 0.0002;
    //public static double rotateP = 0, rotateI = 0, rotateD = 0;
    //public static double tiltP = 0, tiltI = 0, tiltD = 0;

    public static int slideTarget=0, rotateTarget=0, tiltTarget=0;
    public static int slideTarget2=0, rotateTarget2=0, tiltTarget2=0;

    @Override
    public void init(){
        PhotonCore.enable();

        tilt  = hardwareMap.get(DcMotor.class,"tilt_arm");
        tilt_claw = hardwareMap.get(Servo.class, "tilt_claw");
        //tilt = hardwareMap.get(DcMotor.class,"tilt_arm");
        //rotate = hardwareMap.get(DcMotor.class,"rotate_arm");
        //tilt_arm = new TurretMotor(tiltP, tiltI, tiltD, tilt);
        //rotate_arm = new TurretMotor(rotateP, rotateI, rotateD, rotate);

        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt_arm = new TurretMotor(tiltP, tiltI,tiltD, tilt);

        tilt_claw.setPosition(0.59);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop(){
        if (gamepad1.a){
            tilt_arm.setTargetPosition(tiltTarget);
        }
        else if (gamepad1.y){
            tilt_arm.setTargetPosition(tiltTarget2);
        }
        //tilt_arm.updateConstants(tiltP, tiltI, tiltD);
        //rotate_arm.updateConstants(rotateP, rotateI, rotateD);
        tilt_claw.setPosition(0.59);
        tilt_arm.updateConstants(tiltP, tiltI,tiltD);

        //tilt_arm.setTargetPosition(tiltTarget);
        //rotate_arm.setTargetPosition(rotateTarget);

        //tilt_arm.toPosition();
        //rotate_arm.toPosition();
        tilt_arm.toPosition();

        //telemetry.addData("tilt pos ", tilt.getCurrentPosition());
        //telemetry.addData("rotate pos ", rotate.getCurrentPosition());
        telemetry.addData("tilt pos ", tilt.getCurrentPosition());
        telemetry.addData("tilt target ", tiltTarget);
        telemetry.addData("tilt target 2",tiltTarget2);
    }

}
