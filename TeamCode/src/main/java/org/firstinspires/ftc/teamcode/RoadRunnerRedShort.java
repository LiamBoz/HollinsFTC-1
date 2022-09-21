package org.hollins.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.hollins.ftc.teamcode.drive.SampleMecanumDrive;
import org.hollins.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "drive")
public class RoadRunnerRedShort extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12, -63, Math.toRadians(90)));
        DcMotorEx lift_arm, rotate_arm, intake, spin;;
        Servo lift_servo;
        lift_arm = hardwareMap.get(DcMotorEx.class,"lift_arm");
        rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        spin = hardwareMap.get(DcMotorEx.class,"spin");
        lift_servo   = hardwareMap.get(Servo.class,"lift_servo");



        TrajectorySequence onlyCyclesRed = drive.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(90)))
                // begins lifting arm to top level
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {lift_arm.setPower(0.5);})
                // 0.5 seconds after that sets the power to 0.175 to hold the arm in the air
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {lift_arm.setPower(0.175);})
                // drives forward to the hub
                .lineToLinearHeading(new Pose2d(0,-38,Math.toRadians(130)))
                // spits out preload block
                .addTemporalMarker(() -> intake.setPower(-0.4))
                // waits time to spit it out
                .waitSeconds(0.5)
                .setReversed(true)
                // splines into the depot
                .splineToSplineHeading(new Pose2d(30,-65.5,Math.toRadians(180)),Math.toRadians(0))
                .lineTo(new Vector2d(42,-65.5))
                // drops the arm to the ground
                .addTemporalMarker(() -> lift_arm.setPower(0))
                // turns 180 for the arm to pick up
                .turn(Math.toRadians(180))
                // picks up block while driving forward into depot
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {intake.setPower(1);})
                .lineTo(new Vector2d(50,-65.5))
                // does this for 0.5 seconds to give time to pick up
                .waitSeconds(0.5)
                // backs away from blocks
                .back(10)
                // strafes to the wall to ensure going through the gap
                .strafeRight(6)
                // backs out of depot
                .back(53)
                // begins lifting arm to 3rd level and turns 90 degrees to the hub
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {lift_arm.setPower(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.525, () -> {lift_arm.setPower(0.175);})
                .turn(Math.toRadians(90))
                // drives forward to the hub
                .forward(18)
                // spits out the block, waiting 1.5 seconds to make sure its out
                .addTemporalMarker(() -> intake.setPower(-0.4))
                .waitSeconds(1.5)
                // backs up & turns towards the depot
                .back(18)
                .turn(Math.toRadians(-90))
                // strafes into the wall to ensure going through the gap
                .strafeRight(10)
                // drives forward into the depot and strafes to the left so our teammate can park
                .forward(56)
                .strafeLeft(30)

                //.splineTo(new Vector2d(0,38),Math.toRadians(-120))
                //.waitSeconds(0.5)
                //.splineToSplineHeading(new Pose2d(30,65.5,Math.toRadians(180)),Math.toRadians(0))
                //.lineTo(new Vector2d(44,65.5))
                //.lineTo(new Vector2d(30,65.5))
                .build();


        waitForStart();
        if(isStopRequested()) return;
        drive.followTrajectorySequence(onlyCyclesRed);
    }
}
