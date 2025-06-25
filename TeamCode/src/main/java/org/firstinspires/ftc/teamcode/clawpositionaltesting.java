package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="clawpositionaltest", group="Iterative Opmode")
public class clawpositionaltesting extends OpMode {
    private Servo claw;
    private Servo tilt_claw;
    private Servo sensor_servo;
    private DcMotor tilt_arm;
    private Servo odometry_forward;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    double servo_position = 0.5;

    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class,"claw");
        tilt_claw = hardwareMap.get(Servo.class,"tilt_claw");
        tilt_arm = hardwareMap.get(DcMotor.class,"tilt_arm");
        sensor_servo = hardwareMap.get(Servo.class, "sensor_servo");
        odometry_forward = hardwareMap.get(Servo.class, "odometry_forward");

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

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

        tilt_arm.setPower(gamepad1.right_stick_y);

        telemetry.addData("servo position",servo_position);

        sensor_servo.setPosition(servo_position);

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            servo_position = servo_position + 0.02;
        }
        else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
            servo_position = servo_position - 0.02;
        }
    }
}
