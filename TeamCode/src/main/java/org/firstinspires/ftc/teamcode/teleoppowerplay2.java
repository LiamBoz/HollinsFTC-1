package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="teleoppowerplay2", group="Iterative Opmode")
public class teleoppowerplay2 extends OpMode {

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private static DcMotor tilt_arm;
    private static DcMotor slide_extension;
    private DcMotor rotate_arm;

    public int ZeroDegreeTiltTicks = 30;
    public int SixtyDegreeTiltTicks = 250;
    public int EightyFiveDegreeTiltTicks = 280;
    public int tilt_ticks;
    public int extension_ticks;
    public double changing_tilt_ticks = 0;
    public int rotation_ticks = 0;
    private Servo claw = null;
    static int MaxPositionTicks = 2250;
    int MinPositionTicks = 0;

    @Override
    public void init() {

        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        slide_extension  = hardwareMap.get(DcMotor.class,"slide_extension");
        tilt_arm = hardwareMap.get(DcMotor.class,"tilt_arm");
        claw = hardwareMap.get(Servo.class,"claw");
        rotate_arm = hardwareMap.get(DcMotor.class,"rotate_arm");
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


        //claw         = hardwareMap.get(Servo.class,"claw");

        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        //tilt_arm.setPower(-gamepad2.right_stick_y);
        rotate_arm.setPower(gamepad2.left_stick_x);
        tilt_ticks = tilt_arm.getCurrentPosition();
        extension_ticks = slide_extension.getCurrentPosition();
        rotation_ticks = rotate_arm.getCurrentPosition();
        changing_tilt_ticks = changing_tilt_ticks + (0.5*(-gamepad2.right_stick_y));
        telemetry.addData("changing tilt ticks:",changing_tilt_ticks);
        telemetry.addData("changing rotation ticks", rotation_ticks);
        tilt_arm.setPower(1);
        tilt_arm.setTargetPosition((int)(changing_tilt_ticks));

        //power = (power + (power-(gamepad2.right_stick_y))/10)*gamepad2.right_stick_y;
        //power = ((1/(gamepad2.right_stick_y))*0.08)*gamepad2.right_stick_y;
        //power = ((gamepad2.right_stick_y)+(gamepad2.right_stick_y/Math.abs(gamepad2.right_stick_y))*Math.abs((0.5)*gamepad2.right_stick_y-power));

        if (gamepad2.y) {
            slide_extension.setPower(1);
            slide_extension.setTargetPosition(MaxPositionTicks);
        }
        else if (gamepad2.a) {
            slide_extension.setPower(1);
            slide_extension.setTargetPosition(MinPositionTicks);
        }
        else {
        }

        if (gamepad2.dpad_right){
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

        }

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

        //slide_extension.setPower(gamepad2.left_stick_y);

        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }
}

