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
public class RoadRunnerRedLong extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx lift_arm, rotate_arm, intake, spin;;
        Servo lift_servo;
        lift_arm = hardwareMap.get(DcMotorEx.class,"lift_arm");
        rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        spin = hardwareMap.get(DcMotorEx.class,"spin");
        lift_servo   = hardwareMap.get(Servo.class,"lift_servo");

        drive.setPoseEstimate(new Pose2d(-35,-61,Math.toRadians(-270)));


        TrajectorySequence onlyCyclesBlue = drive.trajectorySequenceBuilder(new Pose2d(-35, -61, Math.toRadians(-270)))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {lift_arm.setPower(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {lift_arm.setPower(0.175);})
                .splineTo(new Vector2d(-23, -37), Math.toRadians(-310))
                .addTemporalMarker(() -> intake.setPower(-0.3))
                .addTemporalMarker(() -> lift_servo.setPosition(0.79))
                .waitSeconds(0.5)
                .back(10)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .turn(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(-74, -54, Math.toRadians(0)), Math.toRadians(0))
                .turn(Math.toRadians(-90))
                .forward(16)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {spin.setPower(0.35);})
                .waitSeconds(2.5)
                .back(24)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {spin.setPower(0);})
                .addTemporalMarker(() -> lift_arm.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {lift_arm.setPower(0.175);})
                .addTemporalMarker(() -> lift_servo.setPosition(0.375))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .build();

        waitForStart();
        if(isStopRequested()) return;
        drive.followTrajectorySequence(onlyCyclesBlue);
    }
}
