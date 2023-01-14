package org.firstinspires.ftc.teamcode;

public class PickPlaceOptions {


    // HP LEft Values
    public int rotate_collect;
    public int tilt_collect = 655;
    public int slide_collect = 1155;
    public int rotate_drop = 240;
    public int tilt_drop = -2316;
    public int slide_drop = 1360;
    public int slide_var = 0;
    public double CLAW_HOLD = 0.0;
    public double CLAW_DEPOSIT = 0.35;
    public double CLAWTILT_COLLECT = 0.62;
    public double CLAWTILT_DEPOSIT = 0.72;



    public PickPlaceOptions() {
        this.rotate_collect = 1122;
        this.rotate_drop = 220;
        this.slide_drop = 1280;
        this.tilt_drop = -2316;
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