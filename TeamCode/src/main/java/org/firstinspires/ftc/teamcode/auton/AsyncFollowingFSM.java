package org.firstinspires.ftc.teamcode.auton;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_slide_ticks;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.variable_tilt_ticks;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "advanced")
public class AsyncFollowingFSM extends LinearOpMode {

    private static DcMotor tilt_arm;
    private static DcMotor slide_extension;

    public static int variable_tilt_ticks1 = 0;
    public static int variable_slide_ticks1 = 0;

    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        IDLE            // Our bot will enter the IDLE state when done
    }
    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(35, 61, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        lift lift = new lift(hardwareMap);

        slide_extension = hardwareMap.get(DcMotorEx.class,"slide_extension");
        tilt_arm = hardwareMap.get(DcMotorEx.class,"tilt_arm");
        slide_extension.setTargetPosition(variable_slide_ticks1);
        slide_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt_arm.setTargetPosition(variable_tilt_ticks1);
        tilt_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_extension.setPower(1);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(36,26))
                .splineTo(new Vector2d(42,10),Math.toRadians(180))
                //.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> variable_slide_ticks1 = 2250)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> variable_slide_ticks1 = 2250)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;
        drive.followTrajectorySequenceAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {


            switch (currentState) {
                case TRAJECTORY_1:

                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                    }
                    break;
                case IDLE:

                    break;
            }

            drive.update();
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    class lift {
        public lift(HardwareMap hardwareMap) {
            slide_extension = hardwareMap.get(DcMotorEx.class,"slide_extension");
            tilt_arm = hardwareMap.get(DcMotorEx.class,"tilt_arm");
            slide_extension.setTargetPosition(variable_slide_ticks1);
            slide_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tilt_arm.setTargetPosition(variable_tilt_ticks1);
            tilt_arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tilt_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public void update() {
            slide_extension.setPower(1);
            tilt_arm.setPower(1);
            tilt_arm.setTargetPosition(variable_tilt_ticks1);
            slide_extension.setTargetPosition(variable_slide_ticks1);
        }
    }
}