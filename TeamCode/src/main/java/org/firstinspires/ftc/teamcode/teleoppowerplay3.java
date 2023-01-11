package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auton.FSMAutoShortPole;

@TeleOp(name="teleoppowerplay3", group="Iterative Opmode")
public class teleoppowerplay3 extends OpMode {

    ElapsedTime liftTimer = new ElapsedTime();

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private static DcMotor tilt_arm;
    private static DcMotor slide_extension;
    private DcMotor rotate_arm;

    public int rotate_collect = 860;
    public int tilt_collect = 0;
    public int slide_collect = 1200;
    public int rotate_drop = -368;
    public int tilt_drop = 580;
    public int slide_drop = 1660;
    public int slide_var = 0;
    public double CLAW_HOLD = 0.0;
    public double CLAW_DEPOSIT = 0.13;
    final double CLAWTILT_COLLECT = 0.5;
    final double CLAWTILT_DEPOSIT = 0.75;

    double odometry_forward_static = 0.5;
    double odometry_strafe_static = 0.5;

    int tilt_position = 1;
    int slide_position = 0;

    double tiltclaw_4 = 0.9;
    double tiltclaw_3 = 0.78;
    double tiltclaw_2 = 0.7;
    double tiltclaw_0 = 0.65;

    public int ZeroDegreeTiltTicks = 30;
    public int SixtyDegreeTiltTicks = 250;
    public int EightyFiveDegreeTiltTicks = 280;
    public int tilt_ticks;
    public int extension_ticks;
    public double changing_tilt_ticks = 0;
    public int rotation_ticks = 0;



    double target;
    double error;
    double Kp = 0.04;
    double leftPow;
    double rightPow;

    boolean slidevar = true;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    double botHeading;

    BNO055IMU imu;
    double globalAngle, power = .30, correction;

    private Servo claw = null;
    private Servo tilt_claw = null;
    private Servo odometry_forward = null;
    private Servo odometry_strafe = null;
    static int MaxPositionTicks = 1400;
    int MinPositionTicks = 0;

    @Override
    public void init() {

        PhotonCore.enable();

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

// Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
// Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
// Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

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
        slide_extension.setDirection(DcMotor.Direction.REVERSE);

        slide_extension.setTargetPosition(0);
        tilt_arm.setTargetPosition(0);
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

        //claw.setPosition(CLAW_DEPOSIT);
        //tilt_claw.setPosition(CLAWTILT_COLLECT);
/*        odometry_forward.setPosition(0.55);
        odometry_strafe.setPosition(0.2);*/
        slide_extension.setPower(1);
        tilt_arm.setPower(1);
        //claw         = hardwareMap.get(Servo.class,"claw");
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

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


        if (gamepad1.left_bumper){
            claw.setPosition(0.7);
        }
        else if (gamepad1.right_bumper){
            claw.setPosition(0.35);
        }
/*        if (gamepad1.dpad_up){
            tilt_claw.setPosition(0.3);
        }
        else if (gamepad1.dpad_down){
            tilt_claw.setPosition(0.78);
        }*/

        //tilt_ticks = tilt_arm.getCurrentPosition();
        //extension_ticks = slide_extension.getCurrentPosition();
        rotation_ticks = rotate_arm.getCurrentPosition();
        //telemetry.addData("changing tilt ticks:",changing_tilt_ticks);
        telemetry.addData("changing rotation ticks", rotation_ticks);
        telemetry.addData("tilt ticks", tilt_arm.getCurrentPosition());
        telemetry.addData("rotatre goal", MinPositionTicks);
        telemetry.addData("lifttimer", liftTimer.seconds());
        telemetry.addData("stuff", Math.abs(slide_extension.getCurrentPosition() - slide_collect));
        telemetry.addData("odometry_forward", odometry_forward.getPosition());
        telemetry.addData("odometry_strafe", odometry_strafe.getPosition());
        telemetry.addData("SLDIEVAR",slidevar);

        botHeading = imu.getAngularOrientation().firstAngle;

        //rotate_arm.setPower(-gamepad1.right_stick_x*0.65);
        MinPositionTicks += (-gamepad1.right_stick_x*10);


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



        rotate_arm.setPower(1);
        //tilt_arm.setPower(0.5);
        //slide_extension.setPower(1);




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
        telemetry.addData("target angle", target);
        telemetry.addData("angle error", error);
        telemetry.addData("botheading", Math.toDegrees(botHeading));
        //telemetry.addData("encoder ticks for tilt",tilt_ticks);

        //slide_extension.setPower(gamepad2.left_stick_y);

        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

        //target += -gamepad1.right_stick_x*50;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double frontLeftPower = (y + x) / denominator;
        double backLeftPower = (y - x) / denominator;
        double frontRightPower = (y - x) / denominator;
        double backRightPower = (y + x) / denominator;

        //while((Math.abs(error)>2))
        //{
        error = -Math.toDegrees(botHeading) - target;
        frontLeftPower += error * Kp;
        backLeftPower += error * Kp;
        frontRightPower += -error * Kp;
        backRightPower += -error * Kp;
            //setMotorPowers(leftPow, rightPow);
        //}

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

/*
        if (angleTank2Enemy > angleCannon) {

            diffAngle = angleTank2Enemy - angleCannon;
            if (diffAngle > MathUtils.PI) {
                rotDirRight = true;
            }
            else rotDirRight = false;
        }
        else {
            diffAngle = angleCannon - angleTank2Enemy;
            if (diffAngle > MathUtils.PI) {
                rotDirRight = false;
            }
            else rotDirRight = true;
        }

        if ((diffAngle) > 0.05f) {  // dead zone

            if (rotDirRight)    direction = 1;  // this goes to cannon.getBody().setAngularVelocity(cannonAngularVelocity);
            else                direction = -1; // this goes to cannon.getBody().setAngularVelocity(cannonAngularVelocity);
        }
*/



    }
}

