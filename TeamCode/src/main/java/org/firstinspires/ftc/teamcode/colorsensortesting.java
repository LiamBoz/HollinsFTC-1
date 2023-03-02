package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_slide_ticks;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_tilt_ticks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class colorsensortesting extends OpMode {
    // Define a variable for our color sensor

    public enum PoleSearch {
        POLE_BEGIN,
        POLE_SEARCH,
        POLE_FOUND,
        POLE_FINISH
    }

    PoleSearch poleSearch = PoleSearch.POLE_BEGIN;

    DistanceSensor colorsensor1;
    Servo tilt_claw;
    Servo claw;
    DcMotor slide_extension;
    DcMotor tilt_arm;
    DcMotor rotate_arm;
    Servo sensor_servo;
    boolean IsPoleDetected = false;

    ElapsedTime PoleSearchTimer = new ElapsedTime();

    double RotateArmBegin = 50;
    double RotateArmPosition = 0;

    @Override
    public void init() {

        // Get the color sensor from hardwareMap
        colorsensor1 = hardwareMap.get(DistanceSensor.class, "colorsensor1");
        tilt_claw = hardwareMap.get(Servo.class, "tilt_claw");
        slide_extension = hardwareMap.get(DcMotor.class,"slide_extension");
        claw = hardwareMap.get(Servo.class, "claw");
        tilt_arm = hardwareMap.get(DcMotor.class, "tilt_arm");
        rotate_arm = hardwareMap.get(DcMotor.class,"rotate_arm");
        sensor_servo = hardwareMap.get(Servo.class, "sensor_servo");
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
        slide_extension.setPower(1);
        tilt_arm.setPower(1);
        rotate_arm.setPower(1);

        tilt_claw.setPosition(0.7);
        claw.setPosition(0.0);
        sensor_servo.setPosition(0.68);

    }

    @Override
    public void loop() {

        /*if (gamepad1.a){
            poleSearch = PoleSearch.POLE_FINISH;
        }

        switch (poleSearch){
            case POLE_BEGIN:
*//*                PoleSearchTimer.reset();
                rotate_arm.setTargetPosition(0);
                slide_extension.setTargetPosition(430);
                tilt_arm.setTargetPosition(-1580);*//*
                if (gamepad1.b){
                    poleSearch = PoleSearch.POLE_SEARCH;
                    break;
                }
                break;

            case POLE_SEARCH:
                rotate_arm.setTargetPosition((int)RotateArmPosition);
                RotateArmPosition = RotateArmBegin + 100*PoleSearchTimer.seconds();
                if (colorsensor1.getDistance(DistanceUnit.INCH) <= 2.2){
                    poleSearch = PoleSearch.POLE_FOUND;
                    break;
                }
                break;
            case POLE_FOUND:{
                PoleSearchTimer.reset();
                tilt_claw.setPosition(1);
                break;
            }
            case POLE_FINISH:{
                slide_extension.setTargetPosition(0);
                rotate_arm.setTargetPosition(0);
                tilt_arm.setTargetPosition(0);
            }
        }*/



        telemetry.addData("distance",colorsensor1.getDistance(DistanceUnit.INCH));
        telemetry.addData("rotate position", RotateArmPosition);
        telemetry.addData("lifttimer", PoleSearchTimer.seconds());

        //slide_extension.setTargetPosition(0);

    }
}