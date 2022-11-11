package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_slide_ticks;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_tilt_ticks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="FSM AUTO SHORT POLE EXTENDABLE")
public class FSMAutoShortPoleExtendable extends OpMode {

    public enum LiftState {
        LIFT_STARTDROP,
        LIFT_GETNEW,
        LIFT_GETNEWRETRACT,
        LIFT_DROP,
        PARKING_STATE
    }

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_STARTDROP;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    TrajectorySequence BlueOnRedGoMiddle;
    TrajectorySequence BlueOnRedGoRight;
    TrajectorySequence BlueOnRedGoLeft;

    public DcMotorEx slide_extension;
    public DcMotorEx tilt_arm;
    public DcMotorEx rotate_arm;
    public Servo claw;
    public Servo tilt_claw;

    ElapsedTime liftTimer = new ElapsedTime();

    SampleMecanumDrive drive;

    int cones_dropped = 0;
    int CONES_DESIRED = 3;

    boolean switchvar = true;

    final double CLAW_HOLD = 0.2; // the idle position for the dump servo
    final double CLAW_DEPOSIT = 0.0; // the dumping position for the dump servo

    final double CLAWTILT_COLLECT = 0.5;
    final double CLAWTILT_DEPOSIT = 0.6;

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME = 1;
    final double ROTATE_TIME = 0.3; // the amount of time it takes to rotate 135 degrees
    final double EXTENSION_TIME = 0.6; // e amount of time it takes to extend from 0 to 2250 on the slide

    final int SLIDE_LOW = 0; // the low encoder position for the lift
    final int SLIDE_COLLECT = 1435; // the high encoder position for the lift
    final int SLIDE_DROPOFF = 1350;
    final int SLIDE_MOVEMENT = 1125; // the slide retraction for when rotating

    // TODO: find encoder values for tilt
    final int TILT_LOW = 100;
    final int TILT_HIGH = 450;

    // TODO: find encoder values for rotation
    final int ROTATE_COLLECT = -2235;
    final int ROTATE_DROP = -1200;

    //public TrajectorySequence VariablePath;

    public void init() {
        liftTimer.reset();

        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(36, 63, Math.toRadians(270)));
        slide_extension = hardwareMap.get(DcMotorEx.class,"slide_extension");
        tilt_arm = hardwareMap.get(DcMotorEx.class,"tilt_arm");
        rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");
        claw = hardwareMap.get(Servo.class,"claw");
        tilt_claw = hardwareMap.get(Servo.class,"tilt_claw");

        //rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");

        slide_extension.setDirection(DcMotor.Direction.REVERSE);
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
        tilt_claw.setPosition(CLAWTILT_COLLECT);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        telemetry.setMsTransmissionInterval(50);


        while (tagOfInterest == null)
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
        }

        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        BlueOnRedGoMiddle = drive.trajectorySequenceBuilder(new Pose2d(39,12, Math.toRadians(270)))
                .strafeRight(4)
                .back(24)
                .build();
        BlueOnRedGoRight = drive.trajectorySequenceBuilder(new Pose2d(39,12, Math.toRadians(270)))
                .strafeRight(28)
                .back(24)
                .build();
        BlueOnRedGoLeft = drive.trajectorySequenceBuilder(new Pose2d(39,12, Math.toRadians(270)))
                .strafeLeft(20)
                .back(24)
                .build();

        TrajectorySequence BlueOnRedGoCycle = drive.trajectorySequenceBuilder(new Pose2d(36, 63, Math.toRadians(270)))
                .lineTo(new Vector2d(36,60))
                .addDisplacementMarker(() -> switchvar = true)
                .lineTo(new Vector2d(36,24))
                .splineToConstantHeading(new Vector2d(42,12), Math.toRadians(270))
                .build();

        //drive.followTrajectorySequenceAsync(BlueOnRedGoCycle);
        //drive2.followTrajectorySequenceAsync(VariablePath);




    }

    public void loop() {

        drive.update();

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("encoder ticks for slide",slide_extension.getCurrentPosition());
        telemetry.addData("encoder ticks for tilt",tilt_arm.getCurrentPosition());
        telemetry.addData("rotation ticks", rotate_arm.getCurrentPosition());
        telemetry.addData("claw position", claw.getPosition());
        telemetry.addData("claw tilt", tilt_claw.getPosition());
        telemetry.addData("timer",liftTimer.seconds());
        telemetry.addData("liftstate", liftState);
        telemetry.addData("cones dropped", cones_dropped);
        telemetry.addData("tag location", tagOfInterest);
        //telemetry.update();

        switch (liftState) {
            case LIFT_STARTDROP:
                if (switchvar) {
                    // if liftstate is called, start extending
                    tilt_arm.setPower(1);
                    tilt_arm.setTargetPosition(TILT_HIGH);
                    rotate_arm.setPower(0.5);
                    rotate_arm.setTargetPosition(ROTATE_DROP);
                    tilt_claw.setPosition(CLAWTILT_DEPOSIT);
                    if (Math.abs(rotate_arm.getCurrentPosition() - ROTATE_DROP) <= 5) {
                            slide_extension.setPower(1);
                            slide_extension.setTargetPosition(SLIDE_DROPOFF);
                            if (Math.abs(slide_extension.getCurrentPosition() - SLIDE_DROPOFF) <= 10) {
                                liftState = LiftState.LIFT_DROP;
                            }
                        }
                    liftTimer.reset();
                }
                break;
            case LIFT_GETNEW:
                // check if the left has finished extending,
                // otherwise do nothing.
                //if ((Math.abs(tilt_arm.getCurrentPosition() - TILT_HIGH) < 10) && pathonend > 1 && Math.abs(rotate_arm.getCurrentPosition() - ROTATE_DROP) <= 5)  {
                    // our threshold is within 10 encoder ticks of our target.
                    // set the slide to extend
                if (slide_extension.getCurrentPosition() <= 5)
                    rotate_arm.setTargetPosition(ROTATE_COLLECT);
                    tilt_arm.setTargetPosition(TILT_LOW);
                    tilt_claw.setPosition(CLAWTILT_COLLECT);
                        if (Math.abs(tilt_arm.getCurrentPosition() - TILT_LOW) <= 3){
                            slide_extension.setTargetPosition(SLIDE_COLLECT);
                            liftTimer.reset();
                                if (Math.abs(slide_extension.getCurrentPosition() - SLIDE_COLLECT) <= 5){
                                    claw.setPosition(CLAW_HOLD);
                                        if (claw.getPosition() >= 0.19)
                                                tilt_claw.setPosition(CLAWTILT_COLLECT);
                                                    if (tilt_claw.getPosition() <= 0.51)
                                                            liftState = LiftState.LIFT_GETNEWRETRACT;
                                }
                        }
                    liftTimer.reset();

                //}
                break;
            case LIFT_DROP:
                    tilt_arm.setTargetPosition(TILT_HIGH);
                    rotate_arm.setTargetPosition(ROTATE_DROP);
                if (tilt_arm.getCurrentPosition() - TILT_HIGH <= 5) {
                            slide_extension.setTargetPosition(SLIDE_DROPOFF);
                    if (Math.abs(rotate_arm.getCurrentPosition() - ROTATE_DROP) <= 3){
                        claw.setPosition(CLAW_DEPOSIT);
                        cones_dropped += 1;
                        break;
                    }
                }

            case LIFT_GETNEWRETRACT:
                slide_extension.setTargetPosition(SLIDE_LOW);
                    if (slide_extension.getCurrentPosition() <= 10) {
                        liftState = LiftState.LIFT_GETNEW;
                    }
            case PARKING_STATE:
                if (tagOfInterest == null || tagOfInterest.id == LEFT){

                    drive.followTrajectorySequence(BlueOnRedGoLeft);

                }
                else if (tagOfInterest.id == RIGHT){

                    drive.followTrajectorySequence(BlueOnRedGoRight);

                }
                else if (tagOfInterest.id == MIDDLE){

                    drive.followTrajectorySequence(BlueOnRedGoMiddle);
                }


        }
    }

    // mecanum drive code goes here
    // But since none of the stuff in the switch case stops
    // the robot, this will always run
    //updateDrive(gamepad1, gamepad2);

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    };
}
