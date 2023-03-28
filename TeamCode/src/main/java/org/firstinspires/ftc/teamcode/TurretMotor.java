package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;

public class TurretMotor{
    private int targetPosition;
    private PIDController pidController;
    private DcMotor motor;

    public TurretMotor(double P, double I, double D, DcMotor Motor){
        pidController = new PIDController(P,I,D);
        motor = Motor;
    }
    public void updateConstants(double P, double I, double D){
        pidController.setPID(P, I, D);
    }

    public void toPosition(){
        //if(!(pidController.atSetPoint())){
        double output = pidController.calculate(motor.getCurrentPosition(), targetPosition);
        motor.setPower(output);
        //}
        //else{
        //double output = 0;
        //motor.setPower(output);
        // }

    }


    public void setTargetPosition(int newTargetPosition){
        targetPosition = newTargetPosition;
    }
}
