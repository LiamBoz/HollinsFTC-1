package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_slide_ticks;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_tilt_ticks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;


@Autonomous(group = "drive")
public class ASyncAutoTest extends OpMode{

    private static DcMotor tilt_arm;
    private static DcMotor slide_extension;

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    static class Lift2 {
        public Lift2(HardwareMap hardwareMap) {
            slide_extension.setTargetPosition(variable_slide_ticks);
            slide_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tilt_arm.setTargetPosition(variable_tilt_ticks);
            tilt_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tilt_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public static void update() {
            tilt_arm.setTargetPosition(variable_tilt_ticks);
            slide_extension.setTargetPosition(variable_slide_ticks);
        }
    }

    public void init() {

        //AsyncFollowingFSM.Lift lift2 = AsyncFollowingFSM.Lift(hardwareMap);

        //Trajectory BlueOnRedGoLeft;

        TrajectorySequence BlueOnRedGoLeft = drive.trajectorySequenceBuilder(new Pose2d(35, 61, Math.toRadians(270)))
                .lineTo(new Vector2d(36,26))
                .splineTo(new Vector2d(42,10),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> variable_slide_ticks = 2250)
                .build();

        slide_extension = hardwareMap.get(DcMotorEx.class,"slide_extension");
        tilt_arm = hardwareMap.get(DcMotorEx.class,"tilt_arm");

        slide_extension.setTargetPosition(variable_slide_ticks);
        slide_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt_arm.setTargetPosition(variable_tilt_ticks);
        tilt_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectorySequenceAsync(BlueOnRedGoLeft);


    }

    public void loop() {

        drive.update();

        Lift2.update();
    }
}
