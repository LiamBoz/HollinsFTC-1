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

@Config
@TeleOp(name="turretPIDTuning", group="Iterative Opmode")
public class turretPIDTuning extends OpMode{
    //public TurretMotor tilt_arm;
    //public TurretMotor rotate_arm;
    public TurretMotor rotate_arm;
    public DcMotor slide_extension;

    public DcMotor /*tilt, rotate,*/ rotate;

    public Servo tilt_claw;

    public static double rotateP = 0.0035, rotateI = 0, rotateD = 0.00015;
    //public static double rotateP = 0, rotateI = 0, rotateD = 0;
    //public static double tiltP = 0, tiltI = 0, tiltD = 0;

    public static int slideTarget=0, rotateTarget=0, tiltTarget=0;
    public static int slideTarget2=0, rotateTarget2=0, tiltTarget2=0;

    @Override
    public void init(){
        PhotonCore.enable();

        rotate  = hardwareMap.get(DcMotor.class,"rotate_arm");
        tilt_claw = hardwareMap.get(Servo.class, "tilt_claw");
        slide_extension = hardwareMap.get(DcMotor.class, "slide_extension");
        //tilt = hardwareMap.get(DcMotor.class,"tilt_arm");
        //rotate = hardwareMap.get(DcMotor.class,"rotate_arm");
        //tilt_arm = new TurretMotor(tiltP, tiltI, tiltD, tilt);
        //rotate_arm = new TurretMotor(rotateP, rotateI, rotateD, rotate);

        slide_extension.setTargetPosition(0);
        rotate.setDirection(DcMotorSimple.Direction.REVERSE);
        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate_arm = new TurretMotor(rotateP, rotateI,rotateD, rotate);

        tilt_claw.setPosition(0.59);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop(){
        slide_extension.setPower(1);
        slide_extension.setTargetPosition(0);
        if (gamepad1.a){
            rotate_arm.setTargetPosition(rotateTarget);
        }
        else if (gamepad1.y){
            rotate_arm.setTargetPosition(rotateTarget2);
        }
        //tilt_arm.updateConstants(tiltP, tiltI, tiltD);
        //rotate_arm.updateConstants(rotateP, rotateI, rotateD);
        tilt_claw.setPosition(0.59);
        rotate_arm.updateConstants(rotateP, rotateI,rotateD);

        //tilt_arm.setTargetPosition(tiltTarget);
        //rotate_arm.setTargetPosition(rotateTarget);

        //tilt_arm.toPosition();
        //rotate_arm.toPosition();
        rotate_arm.toPosition();

        //telemetry.addData("tilt pos ", tilt.getCurrentPosition());
        //telemetry.addData("rotate pos ", rotate.getCurrentPosition());
        telemetry.addData("rotate pos ", rotate.getCurrentPosition());
        telemetry.addData("rotate target ", rotateTarget);
        telemetry.addData("rotate target 2",rotateTarget2);
    }

}
