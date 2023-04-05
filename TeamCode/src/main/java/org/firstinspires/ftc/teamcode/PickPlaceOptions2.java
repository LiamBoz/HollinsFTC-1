package org.firstinspires.ftc.teamcode;

public class PickPlaceOptions2 {


    // HP LEft Values
    public int rotate_collect = 875;
    public int tilt_collect = 400;
    public int slide_collect = 540;
    public int rotate_drop = 174;
    public int tilt_drop = -1550;
    public int slide_drop = 435;
    public int slide_drop_medium = 150;
    public int slide_drop_low = 0;
    public int slide_drop_high = 210;
    public int slide_var = 0;
    public double CLAW_HOLD = 0.0;
    public double CLAW_DEPOSIT = 0.12;
    public double CLAWTILT_COLLECT = 0.54;
    public double CLAWTILT_DEPOSIT = 0.61;
    public double CLAWTILT_DROPHIGH = 0.61;
    public double POLEGUIDE_DEPOSIT = 0.54;
    public double POLEGUIDE_REST = 0.1;



    public PickPlaceOptions2() {
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