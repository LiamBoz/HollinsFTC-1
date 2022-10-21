package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
public class CVRoadRunnerBlueLong extends LinearOpMode {

    static Location duckLocation = null;

    OpenCvWebcam webcam;

    Mat mat = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE,
        NOT_FOUND
    }

    // 1280, 800
    static final Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(400, 600));

    static final Rect MIDDLE_ROI = new Rect(
            new Point(450, 0),
            new Point(850, 600));

    static final Rect RIGHT_ROI = new Rect(
            new Point(900, 0),
            new Point(1280, 600));
    static double PERCENT_COLOR_THRESHOLD = 0.4;



    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx lift_arm, rotate_arm, intake, spin;
        Servo lift_servo;
        lift_arm = hardwareMap.get(DcMotorEx.class, "lift_arm");
        rotate_arm = hardwareMap.get(DcMotorEx.class, "rotate_arm");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        spin = hardwareMap.get(DcMotorEx.class, "spin");
        lift_servo = hardwareMap.get(Servo.class, "lift_servo");

        drive.setPoseEstimate(new Pose2d(-35, 61, Math.toRadians(270)));


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new CVRoadRunnerBlueLong.SamplePipeline());
        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // lmao teagan wtf is this error message
                telemetry.addData("Status", "bad bad so bad how could u mess up this catastrophically");
            }
        });

        waitForStart();




        TrajectorySequence BlueLongHigh = drive.trajectorySequenceBuilder(new Pose2d(-35, 61, Math.toRadians(270)))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift_arm.setPower(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift_arm.setPower(0.175);
                })
                .lineToLinearHeading(new Pose2d(-23, 37, Math.toRadians(315)))
                .addTemporalMarker(() -> intake.setPower(-0.3))
                .addTemporalMarker(() -> lift_servo.setPosition(0.79))
                .waitSeconds(3)
                .addTemporalMarker(() -> intake.setPower(0))
                .back(10)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .turn(Math.toRadians(-130))
                .splineToLinearHeading(new Pose2d(-74, 54, Math.toRadians(180)), Math.toRadians(180))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    spin.setPower(-0.35);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    spin.setPower(0);
                })
                .turn(Math.toRadians(-90))
                .back(24)
                .addTemporalMarker(() -> lift_arm.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    lift_arm.setPower(0.175);
                })
                .addTemporalMarker(() -> lift_servo.setPosition(0.375))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .build();

        TrajectorySequence BlueLongMedium = drive.trajectorySequenceBuilder(new Pose2d(-35, 61, Math.toRadians(270)))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift_arm.setPower(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    lift_arm.setPower(0.175);
                })
                .lineToLinearHeading(new Pose2d(-23, 37, Math.toRadians(315)))
                .addTemporalMarker(() -> intake.setPower(-0.3))
                .waitSeconds(3)
                .addTemporalMarker(() -> intake.setPower(0))
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift_arm.setPower(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    lift_arm.setPower(0.175);
                })
                .addTemporalMarker(() -> lift_servo.setPosition(0.79))
                .waitSeconds(1)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .turn(Math.toRadians(-130))
                .splineToLinearHeading(new Pose2d(-74, 54, Math.toRadians(180)), Math.toRadians(180))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    spin.setPower(-0.35);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    spin.setPower(0);
                })
                .turn(Math.toRadians(-90))
                .back(24)
                .addTemporalMarker(() -> lift_arm.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    lift_arm.setPower(0.175);
                })
                .addTemporalMarker(() -> lift_servo.setPosition(0.375))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .build();

        TrajectorySequence BlueLongShort = drive.trajectorySequenceBuilder(new Pose2d(-35, 61, Math.toRadians(270)))

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift_arm.setPower(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    lift_arm.setPower(0.175);
                })
                .lineToLinearHeading(new Pose2d(-23, 37, Math.toRadians(315)))
                .addTemporalMarker(() -> intake.setPower(-0.3))
                .waitSeconds(3)
                .addTemporalMarker(() -> intake.setPower(0))
                .back(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift_arm.setPower(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    lift_arm.setPower(0.175);
                })
                .addTemporalMarker(() -> lift_servo.setPosition(0.79))
                .waitSeconds(1)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .turn(Math.toRadians(-130))
                .splineToLinearHeading(new Pose2d(-74, 54, Math.toRadians(180)), Math.toRadians(180))
                .strafeRight(10)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    spin.setPower(-0.35);
                })
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    spin.setPower(0);
                })
                .turn(Math.toRadians(-90))
                .back(24)
                .addTemporalMarker(() -> lift_arm.setPower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.45, () -> {
                    lift_arm.setPower(0.175);
                })
                .addTemporalMarker(() -> lift_servo.setPosition(0.375))
                .waitSeconds(1.5)
                .addTemporalMarker(() -> lift_arm.setPower(0))
                .build();

        // IDK if java passes by reference or by value sooooo
        Location loc;
        if (duckLocation == Location.LEFT) {
            drive.followTrajectorySequence(BlueLongHigh);
            telemetry.addData("Skystone Location", "left");
        } else if (duckLocation == Location.MIDDLE) {
            drive.followTrajectorySequence(BlueLongMedium);
            telemetry.addData("Skystone Location", "center");
        } else {
            drive.followTrajectorySequence(BlueLongShort);
            telemetry.addData("Skystone Location", "right");
        }


        waitForStart();
        if (isStopRequested()) return;


            }

    public class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;
        CVRoadRunnerBlueLong.Location location;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//            https://alloyui.com/examples/color-picker/hsv.html
            Scalar lowHSV = new Scalar(23, 50, 70);
            Scalar highHSV = new Scalar(32, 255, 255);

            Core.inRange(mat, lowHSV, highHSV, mat);

            Mat left = mat.submat(LEFT_ROI);
            Mat right = mat.submat(RIGHT_ROI);
            Mat middle = mat.submat(MIDDLE_ROI);

            double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
            double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
            double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

            left.release();
            right.release();
            middle.release();

            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
            double rightPercentage = Math.round(rightValue * 100);
            double leftPercentage = Math.round(leftValue * 100);
            double middlePercentage = Math.round(middleValue * 100);
            boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
            boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;


            if (rightPercentage > leftPercentage && rightPercentage > middlePercentage) {
                location = Location.RIGHT;
                duckLocation = Location.RIGHT;
                telemetry.addData("Skystone Location", "right");
            } else if (leftPercentage > rightPercentage && leftPercentage > middlePercentage) {
                location = Location.LEFT;
                duckLocation = Location.LEFT;
                telemetry.addData("Skystone Location", "left");
            } else {
                location = Location.MIDDLE;
                duckLocation = Location.MIDDLE;
                telemetry.addData("Skystone Location", "center");
            }
            telemetry.update();

            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

            Scalar colorStone = new Scalar(255, 0, 0);
            Scalar colorSkystone = new Scalar(0, 255, 0);

            Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? colorSkystone : colorStone);
            Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT ? colorSkystone : colorStone);
            Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE ? colorSkystone : colorStone);
            return mat;
                    }

        @Override
        public void onViewportTapped(){

            viewportPaused = !viewportPaused;

            if(viewportPaused){
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
                }


             }

