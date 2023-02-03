package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_slide_ticks;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_tilt_ticks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.colorsensortesting;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


// adb connect 192.168.43.1:5555

@Autonomous(name="AUTOExtendingMeetLeft")
public class AUTOExtendingMeetLeft extends OpMode {

    public void init_loop(){
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
                        // Here we set the integer we KNOW is is one of the three from the test above
                        parkingTag = tag.id;
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
    }

    public enum LiftState {
        LIFT_TILTTHECLAW,
        LIFT_STARTDROP,
        LIFT_POLESEARCH,
        LIFT_POLESEARCH_REVERSE,
        LIFT_GETNEW,
        LIFT_RETRACTSLIDE,
        LIFT_HOLD,
        LIFT_LETGO,
        LIFT_DROPCYCLE,
        LIFT_INC,
        LIFT_DUNK,
        PARKING_STATE,
        LIFT_WAITSTATE,
        FINISH
    }


    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_TILTTHECLAW;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    // This Integer will be set to a default LEFT if no tag is found
    int parkingTag = LEFT;

    AprilTagDetection tagOfInterest = null;

    TrajectorySequence BlueOnRedGoMiddle;
    TrajectorySequence BlueOnRedGoRight;
    TrajectorySequence BlueOnRedGoLeft;
    TrajectorySequence BlueOnRedGoCycle;

    DistanceSensor colorsensor1;



    public DcMotorEx slide_extension;
    public DcMotorEx tilt_arm;
    public DcMotorEx rotate_arm;
    public Servo claw;
    public Servo tilt_claw;
    public Servo odometry_forward;
    public Servo odometry_strafe;

    Servo sensor_servo;

    //public VoltageSensor voltageSensor;

    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime FailSafeTimer = new ElapsedTime();
    ElapsedTime PoleSearchTimer = new ElapsedTime();

    SampleMecanumDrive drive;

    int cones_dropped = 0;
    int CONES_DESIRED = 4;

    boolean FailSafe = true;
    boolean FailSafe2 = true;

    final double CLAW_HOLD = 0.0; // the idle position for the dump servo
    final double CLAW_DEPOSIT = 0.35; // the dumping position for the dump servo

    final double CLAWTILT_END = 0.13;
    final double CLAWTILT_COLLECT = 0.50;
    final double CLAWTILT_DEPOSIT = .55;

    boolean switchvar = false;
    boolean epic = true;

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME = 1;
    final double ROTATE_TIME = 0.3; // the amount of time it takes to rotate 135 degrees
    final double EXTENSION_TIME = 0.6; // e amount of time it takes to extend from 0 to 2250 on the slide

    double distance_seen = 0.0; // telemetry of the distance sensor

    final int SLIDE_LOW = 0; // the low encoder position for the lift
    int SLIDE_COLLECT = 535; // the high encoder position for the lift
    final int SLIDE_DROPOFF = 380;
    final int SLIDE_MOVEMENT = 1125; // the slide retraction for when rotating

    // TODO: find encoder values for tilt
    int TILT_LOW = 40;
    final int TILT_HIGH = -1570;
    //public int TILT_DECREMENT = 435;

    // TODO: find encoder values for rotation
    final int ROTATE_COLLECT = 607;
    final int ROTATE_DROP = 587;

    final int ROTATE_PAST = -350;

    double RotateArmBegin = -450;
    double RotateArmPosition = RotateArmBegin;
    double RotateArmOffset = 0;
    double RotateArmFinalPosition = -500;

    //public TrajectorySequence VariablePath;

    public void init() {

        FailSafe = true;
        liftTimer.reset();
        //PhotonCore.enable();


        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(270)));

        slide_extension = hardwareMap.get(DcMotorEx.class,"slide_extension");
        tilt_arm = hardwareMap.get(DcMotorEx.class,"tilt_arm");
        rotate_arm = hardwareMap.get(DcMotorEx.class,"rotate_arm");
        claw = hardwareMap.get(Servo.class,"claw");
        tilt_claw = hardwareMap.get(Servo.class,"tilt_claw");

        odometry_forward = hardwareMap.get(Servo.class, "odometry_forward");
        odometry_strafe = hardwareMap.get(Servo.class, "odometry_strafe");

        colorsensor1 = hardwareMap.get(DistanceSensor.class, "colorsensor1");

        sensor_servo = hardwareMap.get(Servo.class, "sensor_servo");
        odometry_forward.setPosition(0.54);
        odometry_strafe.setPosition(0.25);

        //VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

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
        tilt_claw.setPosition(0.1);

        rotate_arm.setPower(1);
        tilt_arm.setPower(1);
        slide_extension.setPower(1);

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


    /*    init_loop();{
         }*/
        //while (tagOfInterest == null)


        BlueOnRedGoRight = drive.trajectorySequenceBuilder(new Pose2d(3.7,-52, Math.toRadians(270)))
                .strafeRight(26)
                .build();
        BlueOnRedGoLeft = drive.trajectorySequenceBuilder(new Pose2d(3.7,-52, Math.toRadians(270)))
                .strafeLeft(20)
                .build();

        BlueOnRedGoCycle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                //.lineTo(new Vector2d(0,-32))
                //.lineTo(new Vector2d(0,-48))
                .splineToConstantHeading(new Vector2d(0, -52), Math.toRadians(270))
                .waitSeconds(30)
                .forward(30)
                //.strafeLeft(3.7)
                .build();

        sensor_servo.setPosition(0.6);

        init_loop();
        drive.followTrajectorySequenceAsync(BlueOnRedGoCycle);

    }

    public void loop() {
        //Pose2d poseEstimate = drive.getPoseEstimate();

/*        if (drive.getPoseEstimate().getY() >= -47){
            drive.setPoseEstimate(new Pose2d(0, poseEstimate.getY(), poseEstimate.getHeading()));
        }*/


        if (FailSafe){
            FailSafeTimer.reset();
            FailSafe = false;
        }

        if (FailSafeTimer.seconds() >= 27.5){
            liftState = LiftState.PARKING_STATE;
        }




        telemetry.addData("x", (drive.getPoseEstimate()).getX());
        //telemetry.addData("x2", poseEstimate.getX());
        telemetry.addData("y", (drive.getPoseEstimate()).getY());
        //telemetry.addData("y2", poseEstimate.getY());
        //telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("encoder ticks for slide",slide_extension.getCurrentPosition());
        telemetry.addData("encoder ticks for tilt",tilt_arm.getCurrentPosition());
        telemetry.addData("rotation ticks", rotate_arm.getCurrentPosition());
        telemetry.addData("claw position", claw.getPosition());
        telemetry.addData("claw tilt", tilt_claw.getPosition());
        telemetry.addData("timer",liftTimer.seconds());
        telemetry.addData("liftstate", liftState);
        telemetry.addData("cones dropped", cones_dropped);
        telemetry.addData("failsafe timer", FailSafeTimer.seconds());
        //telemetry.addData("movedForward", movedForward);
        //telemetry.addData("tag location", tagOfInterest.id);
        telemetry.addData("drive", drive.isBusy());
        telemetry.addData("distance", colorsensor1.getDistance(DistanceUnit.INCH));
        telemetry.addData("Sensor seen",distance_seen);
        telemetry.addData("Final drop rotate", RotateArmFinalPosition);
        if (drive.getPoseEstimate().getY() < -50){
            switchvar = true;
        }

/*        if ((Math.abs(drive.getPoseEstimate().getY() + 52) >= 0.5) && FailSafeTimer.seconds() >= 5 && FailSafe2){
            drive.followTrajectorySequenceAsync(BlueOnRedGoCycle);
            FailSafe2 = false;
        }*/



        //telemetry.update();
        drive.update();
/*
        Pose2d noX = new Pose2d(0, poseEstimate.getY(), poseEstimate.getHeading());
*/

        //drive.updatePoseEstimate();

/*        if ((poseEstimate.getY() <= -47.75) && (poseEstimate.getY()) >= -48 && !(movedForward)) {
            drive.setPoseEstimate(new Pose2d(0, -48, Math.toRadians(270)));
            movedForward = true;
        }*/
        if (gamepad1.a){
            liftState = LiftState.FINISH;
        }


        switch (liftState) {
            case LIFT_TILTTHECLAW:
                tilt_claw.setPosition(CLAWTILT_DEPOSIT);
                liftState = LiftState.LIFT_STARTDROP;
                break;
            case LIFT_STARTDROP:
                tilt_arm.setTargetPosition(TILT_HIGH);
                rotate_arm.setTargetPosition((int)RotateArmFinalPosition);
                // 275 is pole
                if (Math.abs(rotate_arm.getCurrentPosition() - (int)RotateArmFinalPosition) <= 50 && switchvar) {
                    slide_extension.setTargetPosition(SLIDE_DROPOFF);
                    if ((Math.abs(slide_extension.getCurrentPosition() - SLIDE_DROPOFF) <= 15) && (Math.abs(tilt_arm.getCurrentPosition() - TILT_HIGH) <= 30)) {
                        liftTimer.reset();
                        PoleSearchTimer.reset();
                        liftState = LiftState.LIFT_POLESEARCH;
                        break;
                    }
                }
                break;

            case LIFT_POLESEARCH: {
                if (epic) {
                    RotateArmOffset = 100 * PoleSearchTimer.seconds();
                    RotateArmPosition = RotateArmBegin + RotateArmOffset;
                    rotate_arm.setTargetPosition((int) RotateArmPosition);
                    // add 0.3 second pause
                    distance_seen = colorsensor1.getDistance(DistanceUnit.INCH);
                    if (distance_seen <= 10) {
                        liftTimer.reset();
                        RotateArmPosition = RotateArmPosition + 30;
                        RotateArmFinalPosition = RotateArmPosition;
                        liftState = LiftState.LIFT_DUNK;
                        break;
                    }
                    if (RotateArmPosition > -200) {
                        PoleSearchTimer.reset();
                        //slide_extension.setTargetPosition(SLIDE_DROPOFF + 5);
                        RotateArmBegin = RotateArmPosition;
                        liftState = LiftState.LIFT_POLESEARCH_REVERSE;
                        break;
                    }
                }
                else {
                    liftState = LiftState.LIFT_DUNK;
                    break;
                }
                break;
            }
            case LIFT_POLESEARCH_REVERSE: {
                if (epic) {
                    rotate_arm.setTargetPosition((int) RotateArmPosition);
                    RotateArmOffset = 100 * PoleSearchTimer.seconds();
                    RotateArmPosition = RotateArmBegin - RotateArmOffset;
                    distance_seen = colorsensor1.getDistance(DistanceUnit.INCH);
                    if (distance_seen <= 15) {
                        liftTimer.reset();
                        RotateArmPosition = RotateArmPosition - 30;
                        RotateArmFinalPosition = RotateArmPosition;
                        liftState = LiftState.LIFT_DUNK;
                        break;
                    }
                    else if (RotateArmPosition <= ROTATE_PAST){
                        RotateArmFinalPosition = -415;
                        rotate_arm.setTargetPosition(-415);
                        CONES_DESIRED = CONES_DESIRED - 1;
                        liftState = LiftState.LIFT_DUNK;

                    }

                }
                else {
                    liftState = LiftState.LIFT_DUNK;
                    break;
                }
                break;
            }
            case LIFT_DUNK:
                // if (liftTimer.seconds() > 0.3){
                tilt_claw.setPosition(CLAWTILT_DEPOSIT+0.2);
                epic = false;
                liftState = LiftState.LIFT_INC;
                break;
            //   }

            //break;



                /*// Q's DropCycle

            case LIFT_DROPCYCLE:
                tilt_arm.setPower(1);
                if (tilt_arm.getCurrentPosition() < 180) {
                    tilt_arm.setTargetPosition(TILT_HIGH);
                    break;
                }
                rotate_arm.setTargetPosition(ROTATE_DROP);   */

            case LIFT_DROPCYCLE:
                tilt_arm.setTargetPosition(TILT_HIGH);
                if (tilt_arm.getCurrentPosition() <= -200) {
                    slide_extension.setTargetPosition(0);
                    sensor_servo.setPosition(0);
                    if (slide_extension.getCurrentPosition() <= 50) {
                        liftState = LiftState.LIFT_TILTTHECLAW;
                    }
                }
                break;

            case LIFT_GETNEW:
                if (Math.abs(rotate_arm.getCurrentPosition()) - ROTATE_COLLECT <= 50 && Math.abs(tilt_arm.getCurrentPosition() - TILT_LOW) <= 50) {
                    slide_extension.setTargetPosition(SLIDE_COLLECT);
                    tilt_claw.setPosition(0.50);
                    if (slide_extension.getCurrentPosition() >= (SLIDE_COLLECT - 140)) {
                        claw.setPosition(CLAW_HOLD);
                        liftTimer.reset();
                        liftState = LiftState.LIFT_HOLD;
                    }
                }
                break;

            case LIFT_HOLD:
                if (liftTimer.seconds() >= 0.4) {
                    slide_extension.setTargetPosition(SLIDE_COLLECT - 40);
                    tilt_claw.setPosition(0.30);
                    liftState = LiftState.LIFT_DROPCYCLE;
                }
                break;

            case LIFT_INC:
                if (cones_dropped <= CONES_DESIRED) {
                    if (liftTimer.seconds() >= 0.4) {
                        claw.setPosition(CLAW_DEPOSIT);
                        cones_dropped += 1;
                        TILT_LOW = TILT_LOW+70;
                        SLIDE_COLLECT = SLIDE_COLLECT + 2;
                        liftTimer.reset();
                        liftState = LiftState.LIFT_RETRACTSLIDE;
                    }
                }
                else {
                    if (liftTimer.seconds() >= 0.4) {
                        claw.setPosition(CLAW_DEPOSIT);
                        liftTimer.reset();
                        liftState = LiftState.PARKING_STATE;
                    }
                }
                break;
            case LIFT_RETRACTSLIDE:
                slide_extension.setTargetPosition(SLIDE_LOW);
                if (slide_extension.getCurrentPosition() <= 150) {
                    liftTimer.reset();
                    tilt_claw.setPosition(0.50);
                    tilt_arm.setTargetPosition(TILT_LOW);
                    rotate_arm.setTargetPosition(ROTATE_COLLECT);
                    liftState = LiftState.LIFT_GETNEW;
                }
                break;
            case PARKING_STATE:
                liftTimer.reset();
                slide_extension.setTargetPosition(0);
                tilt_claw.setPosition(CLAWTILT_END);
                // Use the parkingTag here - it must be at least LEFT if no tag was seen
                if (parkingTag == LEFT){ //&& cones_dropped >= CONES_DESIRED) {

                    drive.followTrajectorySequenceAsync(BlueOnRedGoLeft);
                    liftTimer.reset();
                    telemetry.addData("left", 1);
                    liftState = LiftState.FINISH;


                } else if (parkingTag == RIGHT){ //&& cones_dropped >= CONES_DESIRED) {

                    drive.followTrajectorySequenceAsync(BlueOnRedGoRight);
                    liftTimer.reset();
                    telemetry.addData("right", 2);
                    liftState = LiftState.FINISH;



                } else if (parkingTag == MIDDLE){ //&& cones_dropped >= CONES_DESIRED) {

                    liftTimer.reset();
                    telemetry.addData("middle", 3);
                    liftState = LiftState.FINISH;

                }
                liftState = LiftState.FINISH;
                break;
            case FINISH:
                FailSafeTimer.reset();
                drive.update();
                slide_extension.setTargetPosition(0);
                tilt_claw.setPosition(CLAWTILT_END);
                if (liftTimer.seconds() >= 0.5) {
                    rotate_arm.setPower(1);
                    rotate_arm.setTargetPosition(0);
                    tilt_arm.setTargetPosition(0);
                }
                break;



        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}