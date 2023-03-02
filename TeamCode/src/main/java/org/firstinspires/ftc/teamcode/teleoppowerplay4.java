package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleoppowerplay4", group="Iterative Opmode")
public class teleoppowerplay4 extends OpMode {

    public enum LiftState {
        LIFT_GRABNEW_SWING,
        LIFT_GRABNEW_GRAB,
        LIFT_CLAWCLOSE,
        LIFT_DROPCONE,
        LIFT_DROPCONEMEDIUM,
        LIFT_EXTENDSLIDE,
        LIFT_RETRACTSLIDE,
        LIFT_CLAWOPEN,
        LIFT_MANUAL_CONTROL,
        LIFT_TILTTHECLAW,
        FINISH
    }

    ElapsedTime liftTimer = new ElapsedTime();

    LiftState liftState = LiftState.LIFT_TILTTHECLAW;

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private static DcMotor tilt_arm;
    private static DcMotor slide_extension;
    private DcMotor rotate_arm;
    private Servo sensor_servo;
    private CRServo lights;


//    public int rotate_collect = 1122;
//    public int tilt_collect = 655;
//    public int slide_collect = 1155;
//    public int rotate_drop = 235;
//    public int tilt_drop = -2316;
//    public int slide_drop = 1498;
//    public int slide_var = 0;
//    public double CLAW_HOLD = 0.35;
//    public double CLAW_DEPOSIT = 0.7;
//    final double CLAWTILT_COLLECT = 0.62;
//    final double CLAWTILT_DEPOSIT = 0.72;

    // Values for HPRight are in the class defaults
    public PickPlaceOptions2 HPRight = new PickPlaceOptions2();
    public PickPlaceOptions2 MPRight = new PickPlaceOptions2();
    public PickPlaceOptions2 LPRight = new PickPlaceOptions2();
    public PickPlaceOptions2 LP2Right = new PickPlaceOptions2();
    public PickPlaceOptions2 HPLeft = new PickPlaceOptions2();
    public PickPlaceOptions2 MPLeft = new PickPlaceOptions2();
    public PickPlaceOptions2 LPLeft = new PickPlaceOptions2();
    public PickPlaceOptions2 LP2Left = new PickPlaceOptions2();
    public PickPlaceOptions2 HPStackLeft = new PickPlaceOptions2();
    public PickPlaceOptions2 HPStackRight = new PickPlaceOptions2();



    // set the values for MPRight




    public PickPlaceOptions2 ActiveOptions = HPRight;

    // set the values for LPRight

    //LPRight.rotate_drop = -966;
    //LPRight.tilt_drop = -826;
    //LPRight.slide_drop = 0;


    double odometry_forward_static = 0.5;
    double odometry_strafe_static = 0.5;

    public int tilt_ticks;
    public int extension_ticks;
    public double changing_tilt_ticks = 0;
    public int rotation_ticks = 0;
    private Servo claw = null;
    private Servo tilt_claw = null;
    private Servo odometry_forward = null;
    private Servo odometry_strafe = null;
    int MinPositionTicks = 0;

    boolean slidevar = true;

    int tilt_position = 1;
    int slide_position = 0;

    double tiltclaw_4 = 0.9;
    double tiltclaw_3 = 0.78;
    double tiltclaw_2 = 0.7;
    double tiltclaw_0 = 0.65;

    int lowest_tiltcollect;
/*    boolean NeedPositionChangeUp = false;
    boolean NeedPositionChangeDown = false;*/

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;
    final double CLAWTILT_END = 0.1;
    final int SLIDE_SWING = 0;

    static int normalpickup = 568;


    double botHeading;

    BNO055IMU imu;
    double globalAngle, power = .30, correction;

    double target = 0;
    double error;
    double Kp = 0.03;
    double leftPow;
    double rightPow;

    //final double CLAWTILT_SWING = 0.5;
    @Override
    public void init() {

        PhotonCore.enable();

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
// Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
// Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);


        MPRight.rotate_drop = -315;
        MPRight.tilt_drop = -1180;
        MPRight.slide_drop = 235;
        MPRight.rotate_collect = 1200;
        MPRight.slide_collect = 500;
        MPRight.tilt_collect = 458;
        MPRight.CLAWTILT_COLLECT = 0.53;
        MPRight.CLAWTILT_DROPHIGH = 0.61;

        LPRight.rotate_drop = -966;
        LPRight.tilt_drop = -600;
        LPRight.slide_drop = 30;
        LPRight.rotate_collect = 1200;
        LPRight.slide_collect = 500;
        LPRight.tilt_collect = 458;
        LPRight.CLAWTILT_COLLECT = 0.53;
        LPRight.CLAWTILT_DROPHIGH = 0.46;

        LP2Right.rotate_drop = -312;
        LP2Right.tilt_drop = -600;
        LP2Right.slide_drop = 120;
        LP2Right.rotate_collect = 1200;
        LP2Right.slide_collect = 500;
        LP2Right.tilt_collect = 458;
        LP2Right.CLAWTILT_COLLECT = 0.53;
        LP2Right.CLAWTILT_DROPHIGH = 0.46;

        HPLeft.rotate_drop = -350;
        HPLeft.tilt_drop = -1483;
        HPLeft.slide_drop = 402;
        HPLeft.rotate_collect = -1270;
        HPLeft.slide_collect = 500;
        HPLeft.tilt_collect = 458;
        HPLeft.CLAWTILT_COLLECT = 0.53;
        HPLeft.CLAWTILT_DROPHIGH = 0.61;

        HPStackLeft.rotate_drop = 220;
        HPStackLeft.tilt_drop = -1520;
        HPStackLeft.slide_drop = 402;
        HPStackLeft.rotate_collect = -1286;
        HPStackLeft.slide_collect = 514;
        HPStackLeft.tilt_collect = 118;
        HPStackLeft.CLAWTILT_COLLECT = 0.58;
        HPStackLeft.CLAWTILT_DROPHIGH = 0.61;

        HPStackRight.rotate_drop = 220;
        HPStackRight.tilt_drop = -1520;
        HPStackRight.slide_drop = 402;
        HPStackRight.rotate_collect = 1223;
        HPStackRight.slide_collect = 514;
        HPStackRight.tilt_collect = 118;
        HPStackRight.CLAWTILT_COLLECT = 0.58;
        HPStackRight.CLAWTILT_DROPHIGH = 0.61;


        MPLeft.rotate_drop = 230;
        MPLeft.tilt_drop = -1180;
        MPLeft.slide_drop = 270;
        MPLeft.rotate_collect = -1270;
        MPLeft.slide_collect = 500;
        MPLeft.tilt_collect = 458;
        MPLeft.CLAWTILT_COLLECT = 0.53;
        MPLeft.CLAWTILT_DROPHIGH = 0.61;

        LPLeft.rotate_drop = 800;
        LPLeft.tilt_drop = -600;
        LPLeft.slide_drop = 50;
        LPLeft.rotate_collect = -1270;
        LPLeft.slide_collect = 500;
        LPLeft.tilt_collect = 458;
        LPLeft.CLAWTILT_COLLECT = 0.53;
        LPLeft.CLAWTILT_DROPHIGH = 0.46;

        LP2Left.rotate_drop = 252;
        LP2Left.tilt_drop = -600;
        LP2Left.slide_drop = 126;
        LP2Left.rotate_collect = -1270;
        LP2Left.slide_collect = 500;
        LP2Left.tilt_collect = 458;
        LP2Left.CLAWTILT_COLLECT = 0.53;
        LP2Left.CLAWTILT_DROPHIGH = 0.46;

/*        ElevatorBot.rotate_drop = -50;
        ElevatorBot.rotate_collect = -1244;
        ElevatorBot.tilt_collect = 600;
        ElevatorBot.tilt_drop = -1800;
        ElevatorBot.slide_collect = 320;
        ElevatorBot.slide_drop = 210;
        ElevatorBot.CLAWTILT_COLLECT = 0.5;
        ElevatorBot.slide_drop_low = 0;
        ElevatorBot.slide_drop_medium = 150;*/


        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        slide_extension  = hardwareMap.get(DcMotor.class,"slide_extension");
        tilt_arm = hardwareMap.get(DcMotor.class,"tilt_arm");
        claw = hardwareMap.get(Servo.class,"claw");
        tilt_claw = hardwareMap.get(Servo.class,"tilt_claw");
        rotate_arm = hardwareMap.get(DcMotor.class,"rotate_arm");
        odometry_forward = hardwareMap.get(Servo.class, "odometry_forward");
        odometry_strafe = hardwareMap.get(Servo.class, "odometry_strafe");
        sensor_servo = hardwareMap.get(Servo.class, "sensor_servo");
        lights = hardwareMap.get(CRServo.class, "lights");
        slide_extension.setDirection(DcMotor.Direction.REVERSE);

        slide_extension.setTargetPosition(MinPositionTicks);
        tilt_arm.setTargetPosition(MinPositionTicks);
        rotate_arm.setTargetPosition(MinPositionTicks);
        slide_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotate_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(ActiveOptions.CLAW_DEPOSIT);
        tilt_claw.setPosition(0.26);
        odometry_forward.setPosition(0.55);
        odometry_strafe.setPosition(0.5);
        sensor_servo.setPosition(0);
        //claw         = hardwareMap.get(Servo.class,"claw");
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        PhotonCore.enable();

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
    }

    @Override
    public void loop() {
        //&& ActiveOptions.tilt_collect >= -50
        //&& ActiveOptions.tilt_collect <= 568
            //lights.setPower(1);


        try {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
        }
        catch (RobotCoreException e) {
            // Swallow the possible exception, it should not happen as
            // currentGamepad1/2 are being copied from valid Gamepads.
        }


        tilt_ticks = tilt_arm.getCurrentPosition();
        extension_ticks = slide_extension.getCurrentPosition();
        rotation_ticks = rotate_arm.getCurrentPosition();
        changing_tilt_ticks = changing_tilt_ticks + (0.5*(-gamepad2.right_stick_y));
        telemetry.addData("changing tilt ticks:",changing_tilt_ticks);
        telemetry.addData("changing rotation ticks", rotation_ticks);
        telemetry.addData("current state", liftState);
        telemetry.addData("claw position", claw.getPosition());
        telemetry.addData("lifttimer", liftTimer.seconds());
        telemetry.addData("stuff", Math.abs(slide_extension.getCurrentPosition() - ActiveOptions.slide_collect));
        telemetry.addData("odometry_forward", odometry_forward.getPosition());
        telemetry.addData("odometry_strafe", odometry_strafe.getPosition());
        telemetry.addData("target", target);
        telemetry.addData("botheading", botHeading);
        telemetry.addData("error", error);
        rotate_arm.setPower(1);
        tilt_arm.setPower(1);
        slide_extension.setPower(1);

        botHeading = imu.getAngularOrientation().firstAngle;
        if (gamepad1.right_stick_x != 0) {
            target = imu.getAngularOrientation().firstAngle;
        }


        if (gamepad1.back){
            liftState = liftState.LIFT_GRABNEW_SWING;
        }
        else if (gamepad1.right_bumper && gamepad1.y){
            // High Pole Right Teleop
            ActiveOptions = HPRight;
            lowest_tiltcollect = HPRight.tilt_collect;
        }
        else if (gamepad1.right_bumper && gamepad1.b){
            // Medium Pole Right Teleop
            ActiveOptions = MPRight;
            lowest_tiltcollect = HPRight.tilt_collect;

        }
        else if (gamepad1.right_bumper && gamepad1.a){
            // Low Pole Right Teleop
            ActiveOptions = LPRight;
            lowest_tiltcollect = HPRight.tilt_collect;

        }
        else if (gamepad1.right_bumper && gamepad1.x){
            // Low Pole Right Teleop
            ActiveOptions = LP2Right;
            lowest_tiltcollect = HPRight.tilt_collect;

        }
        else if (gamepad1.left_bumper && gamepad1.y){
            // Low Pole Right Teleop
            ActiveOptions = HPLeft;
            lowest_tiltcollect = HPRight.tilt_collect;

        }
        else if (gamepad1.left_bumper && gamepad1.b){
            // Low Pole Right Teleop
            ActiveOptions = MPLeft;
            lowest_tiltcollect = HPRight.tilt_collect;

        }
        else if (gamepad1.left_bumper && gamepad1.a){
            // Low Pole Right Teleop
            ActiveOptions = LPLeft;
            lowest_tiltcollect = HPRight.tilt_collect;

        }
        else if (gamepad1.left_bumper && gamepad1.x){
            // Low Pole Right Teleop
            ActiveOptions = LP2Left;
            lowest_tiltcollect = HPRight.tilt_collect;

        }
        else if (gamepad1.left_trigger>0.5){
            liftTimer.reset();
            liftState = liftState.FINISH;
        }
        else if (gamepad1.dpad_right){
            ActiveOptions = HPStackRight;

        }
        else if (gamepad1.dpad_left){
            ActiveOptions = HPStackLeft;

        }


        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
            HPStackLeft.tilt_collect -= 60;
            HPStackRight.tilt_collect -= 60;

            //NeedPositionChangeUp = true;
        }
        else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
            HPStackLeft.tilt_collect += 60;
            HPStackRight.tilt_collect += 60;

            /*if (NeedPositionChangeDown){
                if (ActiveOptions.slide_drop == ActiveOptions.slide_drop_high){
                    ActiveOptions.slide_drop = ActiveOptions.slide_drop_low;
                }
                else if (ActiveOptions.slide_drop == ActiveOptions.slide_drop_medium){
                    ActiveOptions.slide_drop = ActiveOptions.slide_drop_high;
                }
                else{
                    ActiveOptions.slide_drop = ActiveOptions.slide_drop_medium;
                }
                NeedPositionChange = false;*/

        }

/*        if (gamepad1.dpad_down){
            NeedPositionChange = true;
        }
        else{
            if (NeedPositionChange){
                if (ActiveOptions.slide_drop == ActiveOptions.slide_drop_high){
                    ActiveOptions.slide_drop = ActiveOptions.slide_drop_low;
                }
                else if (ActiveOptions.slide_drop == ActiveOptions.slide_drop_medium){
                    ActiveOptions.slide_drop = ActiveOptions.slide_drop_high;
                }
                else{
                    ActiveOptions.slide_drop = ActiveOptions.slide_drop_medium;
                }
                NeedPositionChange = false;
            }
        }*/

        switch (liftState) {
            case LIFT_TILTTHECLAW:
                tilt_claw.setPosition(0.30);
                liftState = LiftState.LIFT_GRABNEW_SWING;

            case LIFT_GRABNEW_SWING:
                if (ActiveOptions == HPStackRight || ActiveOptions == HPStackLeft){
                    tilt_arm.setTargetPosition(ActiveOptions.tilt_collect);
                    ActiveOptions.slide_var = 0;
                    if (slide_extension.getCurrentPosition() <= 50) {
                        claw.setPosition(ActiveOptions.CLAW_DEPOSIT);
                        if (gamepad1.a){
                            rotate_arm.setTargetPosition(ActiveOptions.rotate_collect);
                            tilt_arm.setTargetPosition(ActiveOptions.tilt_collect);
                            tilt_claw.setPosition(ActiveOptions.CLAWTILT_COLLECT);
                            if (Math.abs(rotate_arm.getCurrentPosition() - ActiveOptions.rotate_collect) <= 30){
                                slide_extension.setTargetPosition(ActiveOptions.slide_collect);
                                liftState = LiftState.LIFT_GRABNEW_GRAB;
                            }
                        }
                    }

                }
                else {
                    tilt_arm.setTargetPosition(ActiveOptions.tilt_collect);
                    ActiveOptions.slide_var = 0;
                    if (slide_extension.getCurrentPosition() <= 50) {
                        claw.setPosition(ActiveOptions.CLAW_DEPOSIT);
                        if (gamepad1.a) {
                            rotate_arm.setTargetPosition(ActiveOptions.rotate_collect);
                            tilt_arm.setTargetPosition(ActiveOptions.tilt_collect);
                            if (Math.abs(rotate_arm.getCurrentPosition() - ActiveOptions.rotate_collect) <= 30) {
                                tilt_claw.setPosition(ActiveOptions.CLAWTILT_COLLECT);
                                slide_extension.setTargetPosition(ActiveOptions.slide_collect);
                                liftState = LiftState.LIFT_GRABNEW_GRAB;
                            }
                        }
                    }
                }
                break;
            case LIFT_GRABNEW_GRAB:
                tilt_arm.setTargetPosition(ActiveOptions.tilt_collect);
                if (Math.abs(slide_extension.getCurrentPosition() - ActiveOptions.slide_collect) <= 30 && gamepad1.x) {
                    claw.setPosition(ActiveOptions.CLAW_HOLD);
                    liftTimer.reset();
                    liftState = LiftState.LIFT_CLAWCLOSE;
                }
                break;

            case LIFT_MANUAL_CONTROL:
                MinPositionTicks += (-gamepad1.right_stick_x*10);

                if (gamepad1.left_bumper){
                    claw.setPosition(0.35);
                }
                else if (gamepad1.right_bumper){
                    claw.setPosition(0.0);
                }

                if (currentGamepad1.b && !previousGamepad1.b) {
                    MinPositionTicks = MinPositionTicks - 311;
                }
                else if (currentGamepad1.x && !previousGamepad1.x){
                    MinPositionTicks = MinPositionTicks + 311;
                }


                if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                    tilt_position = tilt_position + 1;
                }
                else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                    tilt_position = tilt_position - 1;
                }

                if (currentGamepad1.y && !previousGamepad1.y) {
                    slidevar = true;
                    slide_position = slide_position + 1;
                }
                else if (currentGamepad1.a && !previousGamepad1.a){
                    slidevar = true;
                    slide_position = slide_position - 1;
                }

                if (tilt_position == 0){
                    tilt_arm.setTargetPosition(900);
                    tilt_claw.setPosition(0.60);
                }
                else if (tilt_position == 1){
                    tilt_arm.setTargetPosition(0);

                }
                else if (tilt_position == 2){
                    tilt_arm.setTargetPosition(-750);
                    tilt_claw.setPosition(tiltclaw_2);
                    if (gamepad1.right_trigger>0.5){
                        tilt_claw.setPosition(tiltclaw_2+0.4);
                    }
                }
                else if (tilt_position == 3){
                    tilt_arm.setTargetPosition(-1400);
                    tilt_claw.setPosition(tiltclaw_3);
                    if (gamepad1.right_trigger>0.5){
                        tilt_claw.setPosition(tiltclaw_3+0.4);
                    }

                }
                else if (tilt_position == 4){
                    tilt_arm.setTargetPosition(-2650);
                    tilt_claw.setPosition(tiltclaw_4);
                    if (gamepad1.right_trigger>0.5){
                        tilt_claw.setPosition(tiltclaw_4+0.4);
                    }
                }
                else if (tilt_position == 5){
                    tilt_arm.setTargetPosition(-3000);
                    tilt_claw.setPosition(tiltclaw_4);
                    if (gamepad1.right_trigger>0.5){
                        tilt_claw.setPosition(tiltclaw_4+0.4);
                    }
                }

                if (gamepad1.dpad_right){
                    tilt_position = 4;
                }
                else if (gamepad1.dpad_left){
                    tilt_position = 0;
                }

                if (slide_position == 0 && slidevar){
                    slide_extension.setTargetPosition(0);
                }
                else if (slide_position == 1 && slidevar){
                    slide_extension.setTargetPosition(550);
                }
                else if (slide_position == 2 && slidevar){
                    slide_extension.setTargetPosition(987);
                }
                else if (slide_position == 3 && slidevar){
                    slide_extension.setTargetPosition(1480);
                }
                if (-gamepad1.right_stick_y < -0.75){
                    slidevar = false;
                    slide_extension.setTargetPosition(0);
                }
                else if(-gamepad1.right_stick_y > 0.75){
                    slidevar = false;
                    slide_extension.setTargetPosition(1480);
                }


                rotate_arm.setTargetPosition(MinPositionTicks);


                break;
            case LIFT_CLAWCLOSE:
                if (liftTimer.seconds() >= 0.23) {
                    liftState = LiftState.LIFT_DROPCONE;
                }
                break;

            case LIFT_DROPCONE:
                if (ActiveOptions == HPStackRight || ActiveOptions == HPStackLeft) {
                    tilt_arm.setTargetPosition(ActiveOptions.tilt_collect - 300);
                    tilt_claw.setPosition(0.4);
                    if (Math.abs(tilt_arm.getCurrentPosition() - (ActiveOptions.tilt_collect - 300)) <= 30){
                    slide_extension.setTargetPosition(SLIDE_SWING);
                    //tilt_claw.setPosition(ActiveOptions.CLAWTILT_DROPHIGH);
                    liftState = LiftState.LIFT_DROPCONEMEDIUM;
                }
                }
                else{
                        slide_extension.setTargetPosition(SLIDE_SWING);
                        tilt_arm.setTargetPosition(ActiveOptions.tilt_drop);
                    //tilt_claw.setPosition(ActiveOptions.CLAWTILT_DEPOSIT);
                        if (slide_extension.getCurrentPosition() <= 30) {
                            tilt_arm.setTargetPosition(ActiveOptions.tilt_drop);
                            rotate_arm.setTargetPosition(ActiveOptions.rotate_drop);
                            tilt_claw.setPosition(ActiveOptions.CLAWTILT_DROPHIGH);
/*                            if (Math.abs(tilt_arm.getCurrentPosition() - ActiveOptions.tilt_drop) <= 100 && Math.abs(rotate_arm.getCurrentPosition() - ActiveOptions.rotate_drop) <= 100){
                                tilt_claw.setPosition(ActiveOptions.CLAWTILT_DEPOSIT);*/
                            if (Math.abs(tilt_arm.getCurrentPosition() - ActiveOptions.tilt_drop) <= 100 && Math.abs(rotate_arm.getCurrentPosition() - ActiveOptions.rotate_drop) <= 100 && gamepad1.a) {
                                liftState = LiftState.LIFT_EXTENDSLIDE;
                            //}
                            }
                        }
                    }
                break;

            case LIFT_DROPCONEMEDIUM:
                if (slide_extension.getCurrentPosition() <= 30) {
                    tilt_claw.setPosition(ActiveOptions.CLAWTILT_DEPOSIT);
                    tilt_arm.setTargetPosition(ActiveOptions.tilt_drop);
                    rotate_arm.setTargetPosition(ActiveOptions.rotate_drop);
                    if (Math.abs(tilt_arm.getCurrentPosition() - ActiveOptions.tilt_drop) <= 100 && Math.abs(rotate_arm.getCurrentPosition() - ActiveOptions.rotate_drop) <= 100 && gamepad1.a) {
                        tilt_claw.setPosition(ActiveOptions.CLAWTILT_DEPOSIT);
                        liftState = LiftState.LIFT_EXTENDSLIDE;
                    }
                }
                break;

            case LIFT_EXTENDSLIDE:
                slide_extension.setTargetPosition(ActiveOptions.slide_drop);
                rotate_arm.setTargetPosition(ActiveOptions.rotate_drop);
                tilt_arm.setTargetPosition(ActiveOptions.tilt_drop);
                if (gamepad1.y){
                    claw.setPosition(0.09);
                }
                else {
                    claw.setPosition(0.0);
                }
                if ((slide_extension.getCurrentPosition() >= (ActiveOptions.slide_drop - 80)) && gamepad1.b) {
                    tilt_claw.setPosition((ActiveOptions.CLAWTILT_DEPOSIT + 0.20));

                    if (gamepad1.right_trigger > 0.5) {
                        claw.setPosition(ActiveOptions.CLAW_DEPOSIT);
                        liftState = LiftState.LIFT_RETRACTSLIDE;
                    }
                } else {
                    tilt_claw.setPosition(ActiveOptions.CLAWTILT_DEPOSIT);
                }
                break;
            case LIFT_RETRACTSLIDE:
                tilt_claw.setPosition(0.4);
                slide_extension.setTargetPosition(0);
                tilt_arm.setTargetPosition(ActiveOptions.tilt_collect);
                if (slide_extension.getCurrentPosition() <= 400){
                    liftState = LiftState.LIFT_TILTTHECLAW;
                }
                break;
            case FINISH:
                slide_extension.setTargetPosition(0);
                tilt_claw.setPosition(CLAWTILT_END);
                if (liftTimer.seconds() >= 0.5) {
                    rotate_arm.setPower(1);
                    rotate_arm.setTargetPosition(0);
                    tilt_arm.setTargetPosition(0);
                }
                break;
        }





        //power = (power + (power-(gamepad2.right_stick_y))/10)*gamepad2.right_stick_y;
        //power = ((1/(gamepad2.right_stick_y))*0.08)*gamepad2.right_stick_y;
        //power = ((gamepad2.right_stick_y)+(gamepad2.right_stick_y/Math.abs(gamepad2.right_stick_y))*Math.abs((0.5)*gamepad2.right_stick_y-power));

/*        if (gamepad2.y) {
            slide_extension.setPower(1);
            slide_extension.setTargetPosition(MaxPositionTicks);
        }
        else if (gamepad2.a) {
            slide_extension.setPower(1);
            slide_extension.setTargetPosition(MinPositionTicks);
        }
        else {
        }*/

/*        if (gamepad2.dpad_right){
            tilt_arm.setPower(1);
            changing_tilt_ticks = 30;
        }
        else if (gamepad2.dpad_left){
            tilt_arm.setPower(1);
            changing_tilt_ticks = 250;
        }
        else if (gamepad2.dpad_down){
            tilt_arm.setPower(1);
            changing_tilt_ticks = 280;
        }
        else{

        }*/

        //claw.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

        /*if (gamepad2.right_bumper)
            claw.setPosition(1);
        else if (gamepad2.left_bumper)
            claw.setPosition(0);
        else
            claw.setPosition(0);

         */

        telemetry.addData("encoder ticks for slide",extension_ticks);
        telemetry.addData("encoder ticks for tilt",tilt_ticks);
        telemetry.addData("tilt arm",ActiveOptions.tilt_drop);
        telemetry.addData("slide ext",ActiveOptions.slide_drop);
        telemetry.addData("rotate arm",ActiveOptions.rotate_drop);




        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x*0.4;

        //slide_extension.setPower(gamepad2.left_stick_y);



        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        error = -Math.toDegrees(botHeading) + Math.toDegrees(target);
        frontLeftPower += error * Kp;
        backLeftPower += error * Kp;
        frontRightPower += -error * Kp;
        backRightPower += -error * Kp;

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

    }
}