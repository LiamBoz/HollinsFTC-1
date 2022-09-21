package org.hollins.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import org.hollins.ftc.teamcode.drive.SampleMecanumDrive;
import org.hollins.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RoadRunnerBlueLong extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx lift_arm, rotate_arm, intake, spin;
        Servo lift_servo;
        lift_arm = hardwareMap.get(DcMotorEx.class,"lift_arm");
        rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        spin = hardwareMap.get(DcMotorEx.class,"spin");
        lift_servo   = hardwareMap.get(Servo.class,"lift_servo");

        drive.setPoseEstimate(new Pose2d(-35,61,Math.toRadians(270)));


        TrajectorySequence BlueLongHigh = drive.trajectorySequenceBuilder(new Pose2d(-35, 61, Math.toRadians(270)))

                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {lift_arm.setPower(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {lift_arm.setPower(0.175);})
                .lineToLinearHeading(new Pose2d(-23,37,Math.toRadians(315)))
                .addTemporalMarker(() -> intake.setPower(-0.3))
                .addTemporalMarker(() -> lift_servo.setPosition(0.79))
                .waitSeconds(3)
                .addTemporalMarker(() -> intake.setPower(0))
                .back(10)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .turn(Math.toRadians(-130))
                .splineToLinearHeading(new Pose2d(-74, 54, Math.toRadians(180)), Math.toRadians(180))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {spin.setPower(-0.35);})
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {spin.setPower(0);})
                .turn(Math.toRadians(-90))
                .back(24)
                .addTemporalMarker(() -> lift_arm.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {lift_arm.setPower(0.175);})
                .addTemporalMarker(() -> lift_servo.setPosition(0.375))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .build();


        waitForStart();
        if(isStopRequested())
            return;
        drive.followTrajectorySequence(BlueLongHigh);
    }
}
