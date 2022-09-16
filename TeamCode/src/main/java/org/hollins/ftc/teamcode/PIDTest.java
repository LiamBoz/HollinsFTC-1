package org.hollins.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID Test")
public class PIDTest extends LinearOpMode {
    DcMotorEx motor;

    double integralSum = 0;
    double kP = 0.5;
    double kI = 0.2;
    double kD = 0.1;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    // Range of positions 0 - 90
    int max_angle = 90;
    int commanded_position = 45;
    int encoder_ticks_per_degreee = 10;
    double  dropping_limit_power = .1;
    int start_integral = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            motor.setPower(PIDControl(commanded_position, motor.getCurrentPosition())/encoder_ticks_per_degreee);
        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();

        if (error == 0)
            integralSum = 0;
        if (Math.abs(error) > start_integral)
            integralSum = 0;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        // Convert to 0-1 for motort control

        return output / max_angle + dropping_limit_power;
    }
}
