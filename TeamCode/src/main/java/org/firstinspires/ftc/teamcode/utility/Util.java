package org.firstinspires.ftc.teamcode.utility;

public class Util {

    public Util(){

    }

    public double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

}
