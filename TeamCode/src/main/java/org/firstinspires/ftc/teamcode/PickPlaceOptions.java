package org.firstinspires.ftc.teamcode;

public class PickPlaceOptions {


    // HP LEft Values
    public int rotate_collect = 1108;
    public int tilt_collect = 568;
    public int slide_collect = 400;
    public int rotate_drop = 220;
    public int tilt_drop = -1403;
    public int slide_drop = 402;
    public int slide_var = 0;
    public double CLAW_HOLD = 0.0;
    public double CLAW_DEPOSIT = 0.35;
    public double CLAWTILT_COLLECT = 0.45;
    public double CLAWTILT_DEPOSIT = 0.55;



    public PickPlaceOptions() {
    }


    public void setTilt_drop(int tilt_drop) {
        this.tilt_drop = tilt_drop;
    }

    public void setSlide_drop(int slide_drop) {
        this.slide_drop = slide_drop;
    }

    public void setRotate_drop(int rotate_drop) {
        this.rotate_drop = rotate_drop;
    }
}