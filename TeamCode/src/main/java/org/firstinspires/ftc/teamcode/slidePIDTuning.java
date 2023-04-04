package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@TeleOp(name="slidePIDTuning", group="Iterative Opmode")
public class slidePIDTuning extends OpMode{
    //public TurretMotor tilt_arm;
    //public TurretMotor rotate_arm;
    public TurretMotor slide_extension;

    public DcMotor /*tilt, rotate,*/ slide;

    public Servo tilt_claw;


    public static double slideF = -0.3;

    public static double slideP = 0.015, slideI = 0.001, slideD = 0.003;
    //public static double rotateP = 0, rotateI = 0, rotateD = 0;
    //public static double tiltP = 0, tiltI = 0, tiltD = 0;

    public static int slideTarget=0, rotateTarget=0, tiltTarget=0;
    public static int slideTarget2=0, rotateTarget2=0, tiltTarget2=0;

    @Override
    public void init(){
        PhotonCore.enable();

        slide  = hardwareMap.get(DcMotor.class,"slide_extension");
        tilt_claw = hardwareMap.get(Servo.class, "tilt_claw");
        //tilt = hardwareMap.get(DcMotor.class,"tilt_arm");
        //rotate = hardwareMap.get(DcMotor.class,"rotate_arm");
        //tilt_arm = new TurretMotor(tiltP, tiltI, tiltD, tilt);
        //rotate_arm = new TurretMotor(rotateP, rotateI, rotateD, rotate);

        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide_extension = new TurretMotor(slideP, slideI,slideD, slide);

        tilt_claw.setPosition(0.59);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop(){
        if (gamepad1.a){
            slide_extension.setTargetPosition(slideTarget);
        }
        else if (gamepad1.y){
            slide_extension.setTargetPosition(slideTarget2);
        }
        //tilt_arm.updateConstants(tiltP, tiltI, tiltD);
        //rotate_arm.updateConstants(rotateP, rotateI, rotateD);
        tilt_claw.setPosition(0.59);
        slide_extension.updateConstants(slideP, slideI,slideD);

        //tilt_arm.setTargetPosition(tiltTarget);
        //rotate_arm.setTargetPosition(rotateTarget);

        //tilt_arm.toPosition();
        //rotate_arm.toPosition();
        slide_extension.updateF(slideF);
        slide_extension.toPosition();

        //telemetry.addData("tilt pos ", tilt.getCurrentPosition());
        //telemetry.addData("rotate pos ", rotate.getCurrentPosition());
        telemetry.addData("slide pos ", slide.getCurrentPosition());
        telemetry.addData("slide target ", slideTarget);
        telemetry.addData("slide target 2",slideTarget2);
    }

}
