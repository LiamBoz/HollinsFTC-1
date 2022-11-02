package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_slide_ticks;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_tilt_ticks;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

@Autonomous(name="FSM AUTO")
public class MoreFSMTest extends OpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }



    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_START;

    public DcMotorEx slide_extension;
    public DcMotorEx tilt_arm;
    public DcMotorEx rotate_arm;
    public Servo claw;

    ElapsedTime liftTimer = new ElapsedTime();

    SampleMecanumDrive drive;

    boolean switchvar = true;
    boolean pathoneend = false;

    final double CLAW_HOLD = 0.16; // the idle position for the dump servo
    final double CLAW_DEPOSIT = 0.0; // the dumping position for the dump servo

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME = 2;
    final double ROTATE_TIME = 0.3; // the amount of time it takes to rotate 135 degrees
    final double EXTENSION_TIME = 0.6; // the amount of time it takes to extend from 0 to 2250 on the slide

    final int SLIDE_LOW = 0; // the low encoder position for the lift
    final int SLIDE_HIGH = 2250; // the high encoder position for the lift
    final int SLIDE_MOVEMENT = 1125; // the slide retraction for when rotating

    // TODO: find encoder values for tilt
    final int TILT_LOW = 30;
    final int TILT_HIGH = 280;

    // TODO: find encoder values for rotation
    final int ROTATE_COLLECT = 890;
    final int ROTATE_DROP = -66;

    public void init() {
        liftTimer.reset();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(35, 61, Math.toRadians(270)));
        slide_extension = hardwareMap.get(DcMotorEx.class,"slide_extension");
        tilt_arm = hardwareMap.get(DcMotorEx.class,"tilt_arm");
        rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");
        claw = hardwareMap.get(Servo.class,"claw");
        //rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");

        //slide_extension.setDirection(DcMotor.Direction.REVERSE);
        slide_extension.setTargetPosition(variable_slide_ticks);
        slide_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt_arm.setTargetPosition(variable_tilt_ticks);
        tilt_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate_arm.setTargetPosition(0);
        rotate_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw.setPosition(CLAW_HOLD);


        TrajectorySequence BlueOnRedGoLeft = drive.trajectorySequenceBuilder(new Pose2d(35, 61, Math.toRadians(270)))
                .lineTo(new Vector2d(36,26))
                .splineTo(new Vector2d(27,11),Math.toRadians(180))
                .build();

        drive.followTrajectorySequenceAsync(BlueOnRedGoLeft);



    }

    public void loop() {

        drive.update();

        switch (liftState) {
            case LIFT_START:
                if (switchvar) {
                    // if liftstate is called, start extending
                    tilt_arm.setPower(1);
                    tilt_arm.setTargetPosition(TILT_HIGH);
                    rotate_arm.setPower(1);
                    rotate_arm.setTargetPosition(ROTATE_DROP);
                    liftState = LiftState.LIFT_EXTEND;
                    liftTimer.reset();
                    switchvar = false;
                }
                break;
            case LIFT_EXTEND:
                // check if the left has finished extending,
                // otherwise do nothing.
                if ((Math.abs(tilt_arm.getCurrentPosition() - TILT_HIGH) < 10) && liftTimer.seconds() >= 2) {
                    // our threshold is within 10 encoder ticks of our target.

                    // set the slide to extend
                    slide_extension.setPower(0);
                    slide_extension.setTargetPosition(SLIDE_HIGH);

                    if (Math.abs(slide_extension.getCurrentPosition()) - SLIDE_HIGH < 15) {

                        claw.setPosition(CLAW_DEPOSIT);
                        liftTimer.reset();
                        liftState = LiftState.LIFT_DUMP;
                    }

                }
                break;
            case LIFT_DUMP:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    //claw.setPosition(CLAW_DEPOSIT);
                    //rotate_turret.setTargetPosition
                    //slide_extension.setTargetPosition(SLIDE_LOW);

                    liftState = LiftState.LIFT_RETRACT;
                }
                break;

            case LIFT_RETRACT:
                if (Math.abs(slide_extension.getCurrentPosition() - SLIDE_LOW) < 10) {
                   // tilt_arm.setTargetPosition(TILT_LOW);
                    //liftState = LiftState.LIFT_START;
                }
                break;

            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_START;


        }
    }

    // mecanum drive code goes here
    // But since none of the stuff in the switch case stops
    // the robot, this will always run!
    //updateDrive(gamepad1, gamepad2);
}
