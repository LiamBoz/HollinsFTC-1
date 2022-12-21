package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class colorsensortesting extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor colorsensor1;
    boolean IsPoleDetected = false;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        colorsensor1 = hardwareMap.get(ColorSensor.class, "colorsensor1");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            if (colorsensor1.blue() >= 20 && colorsensor1.green() >= 20){
                IsPoleDetected = true;
            }
            else{
                IsPoleDetected = false;
            }
            telemetry.addData("Red", colorsensor1.red());
            telemetry.addData("Green", colorsensor1.green());
            telemetry.addData("Blue", colorsensor1.blue());
            telemetry.addData("Is Pole Detected?", IsPoleDetected);
            telemetry.update();
        }
    }
}