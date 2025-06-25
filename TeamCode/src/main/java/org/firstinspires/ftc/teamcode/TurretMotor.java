package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;

public class TurretMotor{
    private int targetPosition;
    private PIDController pidController;
    PIDFController pidfController;
    private DcMotor motor;
    private double F = 0;

    public TurretMotor(double P, double I, double D, DcMotor Motor){
        pidController = new PIDController(P,I,D);
        motor = Motor;
    }

    /*public TurretMotor(double P, double I, double D, double F, DcMotor Motor){
        PIDFController pidfController = new PIDFController(P,I,D,F);
        motor = Motor;
    }*/
    public void updateConstants(double P, double I, double D){
        pidController.setPID(P, I, D);
    }

    /*public void updateConstants(double P, double I, double D, double F){
        pidfController.setPIDF(P, I, D, F);
    }*/

    public void updateF(double newF){
        F = newF;
    }

    public void toPosition(){

        double output = pidController.calculate(motor.getCurrentPosition(), targetPosition);

        //if(!(pidController.atSetPoint())){
            if (pidController.calculate(motor.getCurrentPosition(), targetPosition)+F <= 0){
            motor.setPower((output+F));
        }
            else{
            motor.setPower(output+F);
        }


        return;
        //}
        //else{
        //double output = 0;
        //motor.setPower(output);
        // }

    }

    public void toPositionSlide(){

        double output = pidController.calculate(motor.getCurrentPosition(), targetPosition);

        //if(!(pidController.atSetPoint())){
        if (pidController.calculate(motor.getCurrentPosition(), targetPosition)+F <= 0){
            motor.setPower((output+F/4));
        }
        else{
            motor.setPower(output+F);
        }

    }

   /* public void toPosition(int a){
        double output = pidfController.calculate(motor.getCurrentPosition(), targetPosition);
        motor.setPower(output);
    }*/


    public void setTargetPosition(int newTargetPosition){
        targetPosition = newTargetPosition;
    }
}
