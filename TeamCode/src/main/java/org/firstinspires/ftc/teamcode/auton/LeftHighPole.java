package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_slide_ticks;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_tilt_ticks;
import static org.firstinspires.ftc.teamcode.slidePIDTuning.slideD;
import static org.firstinspires.ftc.teamcode.slidePIDTuning.slideI;
import static org.firstinspires.ftc.teamcode.slidePIDTuning.slideP;
import static org.firstinspires.ftc.teamcode.tiltPIDTuning.tiltD;
import static org.firstinspires.ftc.teamcode.tiltPIDTuning.tiltI;
import static org.firstinspires.ftc.teamcode.tiltPIDTuning.tiltP;
import static org.firstinspires.ftc.teamcode.turretPIDTuning.rotateD;
import static org.firstinspires.ftc.teamcode.turretPIDTuning.rotateI;
import static org.firstinspires.ftc.teamcode.turretPIDTuning.rotateP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TurretMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveTwo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


// adb connect 192.168.43.1:5555


@Config
@Autonomous(name="LeftHighPole")
public class LeftHighPole extends OpMode {

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
        LIFT_GETNEWTILT,
        LIFT_GETNEWSLIDE,
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

    /*TrajectorySequence BlueOnRedGoMiddle;
    TrajectorySequence BlueOnRedGoRight;
    TrajectorySequence BlueOnRedGoLeft;
    TrajectorySequence BlueOnRedGoCycle;
    TrajectorySequence GoForward;
    TrajectorySequence GoBack;
    */


    TrajectorySequence BlueOnRedGoMiddle;
    TrajectorySequence BlueOnRedGoRight;
    TrajectorySequence BlueOnRedGoLeft;
    TrajectorySequence BlueOnRedGoCycle;
    TrajectorySequence BlueOnRedStayStill;
    TrajectorySequence GoForward;
    TrajectorySequence GoBack;
    TrajectorySequence BlueOnRedGoCycleMore;
    TrajectorySequence ParkMiddle;

    //Trajectory initialDriveForward = new

    DistanceSensor colorsensor1;



    //public TurretMotor slide_extension;
    public TurretMotor tilt_arm;
    public TurretMotor rotate_arm;

    public DcMotor slide_extension;
    public DcMotor rotate;
    public DcMotor tilt;

    public Servo claw;
    public Servo tilt_claw;
    public Servo odometry_forward;
    public Servo odometry_strafe;

    Servo sensor_servo;

    //public VoltageSensor voltageSensor;

    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime FailSafeTimer = new ElapsedTime();
    ElapsedTime PreloadTimer = new ElapsedTime();
    ElapsedTime TiltRestTimer = new ElapsedTime();

    SampleMecanumDrive drive;

    int cones_dropped = 0;
    int CONES_DESIRED = 4;

    boolean FailSafe = true;
    boolean FailSafe2 = true;

    final double CLAW_HOLD = 0.0;
    final double CLAW_DEPOSIT = 0.12;

    final double CLAWTILT_END = 0.19;
    final double CLAWTILT_COLLECT = 0.53;
    final double CLAWTILT_DEPOSIT = .59;

    double subtract_tilt = 0;

    boolean switchvar = false;
    boolean epic = true;

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME = 1;
    final double ROTATE_TIME = 0.3; // the amount of time it takes to rotate 135 degrees
    final double EXTENSION_TIME = 0.6; // e amount of time it takes to extend from 0 to 2250 on the slide

    double distance_seen = 0.0; // telemetry of the distance sensor

    final int SLIDE_LOW = 0; // the low encoder position for the lift
    private int SLIDE_COLLECT = 500; // the high encoder position for the lift
    public static int SLIDE_DROPOFF = 500;

    // TODO: find encoder values for tilt
    private int TILT_LOW = -50;
    public static int TILT_HIGH = -1540;

    public double POLEGUIDE_DEPOSIT = 0.5;
    public double POLEGUIDE_REST = 0.13;
    //public int TILT_DECREMENT = 435;

    // TODO: find encoder values for rotation
    final int ROTATE_COLLECT = -8;
    final int ROTATE_DROP = -650;

    boolean TiltRestVar = true;

    boolean drop_preload = false;

    //public static double TILT_P = 0.006, TILT_D = 0.00016, TILT_I = 0.03;
    public static double TILT_P = 0.012, TILT_D = 0.00035, TILT_I = 0.07;
    public static double EXTENDEDTILT_P = 0.1, EXTENDEDTILT_D = 0, EXTENDEDTILT_I = 0;


    //public TrajectorySequence VariablePath;

    public void init() {



        FailSafe = true;
        liftTimer.reset();
        PhotonCore.enable();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(270)));

        slide_extension = hardwareMap.get(DcMotorEx.class,"slide_extension");
        tilt = hardwareMap.get(DcMotorEx.class,"tilt_arm");
        rotate = hardwareMap.get(DcMotorEx.class,"rotate_arm");
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


        slide_extension.setTargetPosition(variable_slide_ticks);
        slide_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide_extension.setPower(1);

        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide_extension.setDirection(DcMotorSimple.Direction.REVERSE);

        claw.setPosition(CLAW_HOLD);
        tilt_claw.setPosition(0.15);


/*        rotate_arm.setPower(1);
        tilt_arm.setPower(1);
        slide_extension.setPower(1);*/

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


        Trajectory initialDriveForward = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(0,-16, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(0,-49, Math.toRadians(360)))

                .build();

    /*    init_loop();{
         }*/
        //while (tagOfInterest == null)


        BlueOnRedGoRight = drive.trajectorySequenceBuilder(new Pose2d(-2,-49, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(-26, -51, Math.toRadians(270)), Math.toRadians(180))
                .back(20)
                //.turn(Math.toRadians(90))
                //.strafeRight(26)
                .build();
        BlueOnRedGoLeft = drive.trajectorySequenceBuilder(new Pose2d(-2,-49, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(23, -51, Math.toRadians(270)), Math.toRadians(180))
                .back(20)
                //.turn(Math.toRadians(90))
                //.strafeLeft(26)
                .build();
        GoForward = drive.trajectorySequenceBuilder(new Pose2d(-26, -49, Math.toRadians(360)))
                .setReversed(false)
                .forward(28)
                .build();
        GoBack = drive.trajectorySequenceBuilder(new Pose2d(2, -49, Math.toRadians(360)))
                .back(28)
                .build();
        ParkMiddle = drive.trajectorySequenceBuilder(new Pose2d(-2,-49, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .back(20)
                .build();




        BlueOnRedGoCycle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                //.splineToConstantHeading(new Vector2d(0,42), Math.toRadians(90))
                //.splineToSplineHeading(new Pose2d(0, 50), Math.toRadians(180))
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(13, -50), Math.toRadians(90))
                //.back(13)
                //.build().
                //.splineToSplineHeading(new Pose2d(0,-40), Math.toRadians(270))
                //.splineTo(new Vector2d(0, -49), Math.toRadians(270))
                //.strafeRight(49)
                //.turn(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0,-20, Math.toRadians(360)), Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(0,-49), Math.toRadians(180))
                .build();

        BlueOnRedGoCycle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                //.splineToConstantHeading(new Vector2d(0,42), Math.toRadians(90))
                //.splineToSplineHeading(new Pose2d(0, 50), Math.toRadians(180))
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(13, -50), Math.toRadians(90))
                //.back(13)
                //.build().
                //.splineToSplineHeading(new Pose2d(0,-40), Math.toRadians(270))
                //.splineTo(new Vector2d(0, -49), Math.toRadians(270))
                //.strafeRight(49)
                //.turn(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, -20, Math.toRadians(270)), Math.toRadians(270))
                .addDisplacementMarker(() -> drive.followTrajectorySequenceAsync(BlueOnRedGoCycleMore))
                //.splineToLinearHeading(new Pose2d(0,-49), Math.toRadians(180))
                .build();

        BlueOnRedGoCycleMore = drive.trajectorySequenceBuilder(new Pose2d(0,-20, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(0,-49, Math.toRadians(360)), Math.toRadians(270))
                .build();
        sensor_servo.setPosition(POLEGUIDE_REST);

        rotate_arm = new TurretMotor(rotateP, rotateI,rotateD, rotate);
        //slide_extension = new TurretMotor(slideP, slideI,slideD, slide);
        tilt_arm = new TurretMotor(tiltP, tiltI,tiltD, tilt);

        tilt_arm.updateConstants(TILT_P, TILT_I, TILT_D);
        rotate_arm.updateConstants(rotateP,rotateI,rotateD);

        init_loop();
        //drive.followTrajectorySequenceAsync(BlueOnRedGoCycle);
        //drive.followTrajectoryAsync(driveForward);
        drive.followTrajectorySequenceAsync(BlueOnRedGoCycle);

    }

    public void loop() {
        //Pose2d poseEstimate = drive.getPoseEstimate();
/*        if(slide_extension.getCurrentPosition()>400){
            slide_extension.setPower(0.759);
        }
        else{slide_extension.setPower(1);}*/

/*        if (drive.getPoseEstimate().getY() >= -47){
            drive.setPoseEstimate(new Pose2d(0, poseEstimate.getY(), poseEstimate.getHeading()));
        }*/

        rotate_arm.toPosition();
        tilt_arm.toPosition();



/*        if (Math.abs(tilt.getCurrentPosition() - TILT_LOW) <= 3){
            TiltRestVar = false;
        }

        else if (TiltRestVar){
            TiltRestTimer.reset();
        }
        else{
            TiltRestVar = true;
        }*/




        if (FailSafe){
            FailSafeTimer.reset();
            FailSafe = false;
        }

        if (FailSafeTimer.seconds() >= 28){
            liftState = LiftState.PARKING_STATE;
        }

        if (drive.getPoseEstimate().getY() <= -48){
            switchvar = true;
        }
        else{
            PreloadTimer.reset();
        }

        if (PreloadTimer.seconds() >= 1){
            drop_preload = true;

        }


        telemetry.addData("x", (drive.getPoseEstimate()).getX());
        //telemetry.addData("x2", poseEstimate.getX());
        telemetry.addData("y", (drive.getPoseEstimate()).getY());
        //telemetry.addData("y2", poseEstimate.getY());
        //telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("encoder ticks for slide",slide_extension.getCurrentPosition());
        telemetry.addData("encoder ticks for tilt",tilt.getCurrentPosition());
        telemetry.addData("tilt target", TILT_LOW);
        telemetry.addData("rotation ticks", rotate.getCurrentPosition());
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
                if (drop_preload) {
                    tilt_claw.setPosition(CLAWTILT_DEPOSIT - 0.1);
                    rotate_arm.updateConstants(rotateP, rotateI, rotateD);
                    liftState = LiftState.LIFT_STARTDROP;
                }
                break;
            case LIFT_STARTDROP:
                tilt_arm.setTargetPosition(TILT_HIGH);
                rotate_arm.setTargetPosition(ROTATE_DROP);
                if ((Math.abs(rotate.getCurrentPosition() - ROTATE_DROP) <= 200)){
                    sensor_servo.setPosition(POLEGUIDE_DEPOSIT);
                    if ((Math.abs(rotate.getCurrentPosition() - ROTATE_DROP) <= 40)) {
                        tilt_claw.setPosition(CLAWTILT_DEPOSIT);
                        slide_extension.setTargetPosition(SLIDE_DROPOFF);
                        if ((Math.abs(slide_extension.getCurrentPosition() - SLIDE_DROPOFF) <= 40) && (Math.abs(tilt.getCurrentPosition() - TILT_HIGH) <= 50)) {
                            liftTimer.reset();
                            liftState = LiftState.LIFT_DUNK;
                            break;
                        }
                    }
                }
                break;


            case LIFT_DUNK:
                if ((Math.abs(rotate.getCurrentPosition() - ROTATE_DROP) <= 50) && (Math.abs(slide_extension.getCurrentPosition() - SLIDE_DROPOFF) <= 40)) {
                    // if (liftTimer.seconds() > 0.3){
                    tilt_claw.setPosition(CLAWTILT_DEPOSIT + 0.1);
                    epic = false;
                    liftTimer.reset();
                    liftState = LiftState.LIFT_INC;
                    break;
                }
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
                TiltRestVar = true;
                if (tilt.getCurrentPosition() <= (TILT_LOW - 200)) {
                    tilt_arm.updateConstants(TILT_P, TILT_I, TILT_D);
                    slide_extension.setTargetPosition(50);
                    //drive.followTrajectorySequenceAsync(GoBack);
                    if (slide_extension.getCurrentPosition() <= 250) {
                        liftState = LiftState.LIFT_TILTTHECLAW;
                        break;
                    }
                }
                else if (cones_dropped >= 5){
                    slide_extension.setTargetPosition(0);
                    if (slide_extension.getCurrentPosition() <= 150) {
                        //drive.followTrajectorySequenceAsync(GoBack);
                        liftState = LiftState.LIFT_TILTTHECLAW;
                        break;
                    }
                }
                break;

            case LIFT_GETNEWTILT:
                if (Math.abs(rotate.getCurrentPosition() - ROTATE_COLLECT) <= 400){
                    tilt_claw.setPosition(CLAWTILT_DEPOSIT);
                    if (Math.abs(rotate.getCurrentPosition() - ROTATE_COLLECT) <= 15 && Math.abs(tilt.getCurrentPosition() - TILT_LOW) <= 30) {
                        //tilt_arm.updateConstants(0,0,0);
                        liftState = LiftState.LIFT_GETNEWSLIDE;
                        break;
                    }
                }
                break;

            case LIFT_GETNEWSLIDE:
                slide_extension.setTargetPosition(SLIDE_COLLECT); /* ret here */
                if (slide_extension.getCurrentPosition() >= (SLIDE_COLLECT - 10)) {
                    claw.setPosition(CLAW_HOLD);
                    //drive.breakFollowing();
                    liftTimer.reset();
                    liftState = LiftState.LIFT_HOLD;
                    break;

                }
                break;

            case LIFT_HOLD:
                if (liftTimer.seconds() >= 0.18) {
                    //slide_extension.setTargetPosition(SLIDE_COLLECT - 40);
                    liftState = LiftState.LIFT_DROPCYCLE;
                    break;
                }
                break;

            case LIFT_INC:
                if (cones_dropped <= CONES_DESIRED) {
                    if (liftTimer.seconds() >= 0.1) {
                        claw.setPosition(CLAW_DEPOSIT);
                        sensor_servo.setPosition(POLEGUIDE_REST);
                        cones_dropped += 1;
                        TILT_LOW = TILT_LOW+60;
                        SLIDE_COLLECT = SLIDE_COLLECT + 0;
                        liftTimer.reset();
                        liftState = LiftState.LIFT_RETRACTSLIDE;
                        break;
                    }
                }
                else {
                    if (liftTimer.seconds() >= 0.1) {
                        claw.setPosition(CLAW_DEPOSIT);
                        liftState = LiftState.PARKING_STATE;
                        break;
                    }
                }
                break;
            case LIFT_RETRACTSLIDE:
                //liftTimer.reset();
                slide_extension.setTargetPosition(SLIDE_LOW);
                //drive.update();
                if (slide_extension.getCurrentPosition() <= 400) {
                    tilt_arm.setTargetPosition(TILT_LOW);
                    tilt_claw.setPosition(CLAWTILT_DEPOSIT - subtract_tilt);
                    if (slide_extension.getCurrentPosition() <= 150) {
                        subtract_tilt = 0.3;
                        //drive.followTrajectorySequenceAsync(GoForward);
                        tilt_claw.setPosition(CLAWTILT_DEPOSIT - subtract_tilt);
                        liftTimer.reset();
                        //tilt_arm.setTargetPosition(TILT_LOW);
                        rotate_arm.setTargetPosition(ROTATE_COLLECT);
                        liftState = LiftState.LIFT_GETNEWTILT;
                        break;
                    }
                }
                break;
            case PARKING_STATE:
                liftTimer.reset();
                FailSafeTimer.reset();
                slide_extension.setTargetPosition(0);
                tilt_claw.setPosition(0.32);
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

                    drive.followTrajectorySequenceAsync(BlueOnRedGoMiddle);
                    liftTimer.reset();
                    telemetry.addData("middle", 3);
                    liftState = LiftState.FINISH;

                }
                liftState = LiftState.FINISH;
                break;
            case FINISH:
                FailSafeTimer.reset();
                drive.update();
                tilt_arm.updateConstants(TILT_P, TILT_I, TILT_D);
                slide_extension.setTargetPosition(0);
                tilt_claw.setPosition(0.2);
                if (liftTimer.seconds() >= 0.5) {
                    //rotate_arm.setPower(1);
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