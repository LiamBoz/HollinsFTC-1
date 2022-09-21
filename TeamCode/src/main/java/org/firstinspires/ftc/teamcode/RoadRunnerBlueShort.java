package org.hollins.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.hollins.ftc.teamcode.drive.SampleMecanumDrive;
import org.hollins.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RoadRunnerBlueShort extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12, 63, Math.toRadians(-90)));
        DcMotorEx lift_arm, rotate_arm, intake, spin;;
        Servo lift_servo;
        lift_arm = hardwareMap.get(DcMotorEx.class,"lift_arm");
        rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        spin = hardwareMap.get(DcMotorEx.class,"spin");
        lift_servo   = hardwareMap.get(Servo.class,"lift_servo");



        TrajectorySequence onlyCyclesBlue = drive.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {lift_arm.setPower(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {lift_arm.setPower(0.175);})
                .lineToLinearHeading(new Pose2d(0,38,Math.toRadians(-130)))
                .addTemporalMarker(() -> intake.setPower(-0.4))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(30,65.5,Math.toRadians(180)),Math.toRadians(0))
                .lineTo(new Vector2d(42,65.5))
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .turn(Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {intake.setPower(1);})
                .lineTo(new Vector2d(50,65.5))
                .waitSeconds(0.5)
                .back(10)
                .strafeLeft(6)
                .back(53)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {lift_arm.setPower(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.525, () -> {lift_arm.setPower(0.175);})
                .turn(Math.toRadians(-90))
                .forward(18)
                .addTemporalMarker(() -> intake.setPower(-0.4))
                .waitSeconds(1.5)
                .back(18)
                .turn(Math.toRadians(90))
                .strafeLeft(10)
                .forward(56)
                .strafeRight(30)
                //.splineTo(new Vector2d(0,38),Math.toRadians(-120))
                //.waitSeconds(0.5)
                //.splineToSplineHeading(new Pose2d(30,65.5,Math.toRadians(180)),Math.toRadians(0))
                //.lineTo(new Vector2d(44,65.5))
                //.lineTo(new Vector2d(30,65.5))
                .build();

        
        waitForStart();
        if(isStopRequested())
            return;
        drive.followTrajectorySequence(onlyCyclesBlue);
    }
}
