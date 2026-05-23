package org.firstinspires.ftc.teamcode.config.utility;

import com.arcrobotics.ftclib.util.InterpLUT;

public class ShooterData {

    public InterpLUT RPMLUT;
    public InterpLUT AngleLUT;
    public InterpLUT ShotTimeLUT;
    public double minDistance = 0;
    public double maxDistance = 1000;

    public ShooterData(){
        RPMLUT = new InterpLUT();
        AngleLUT = new InterpLUT();
        ShotTimeLUT = new InterpLUT();
        addRPMData();
        addAngleData();
        addShotTimeData();
        RPMLUT.createLUT();
        AngleLUT.createLUT();
        ShotTimeLUT.createLUT();
    }

    private void addRPMData(){
        RPMLUT.add(minDistance, 1500);
        RPMLUT.add( 27.5 ,2200); //2100
        RPMLUT.add( 45.6 ,2200);
        RPMLUT.add( 47.2 ,2210);
        RPMLUT.add( 54.6  ,2250);
        RPMLUT.add( 60  ,2300);
        RPMLUT.add( 66.1  ,2360);
        RPMLUT.add( 77.1  ,2400);
        RPMLUT.add( 90  ,2550);
        //Cutoff 100 for close/far
        RPMLUT.add( 115  ,2925);
        RPMLUT.add( 120  ,3000);
        RPMLUT.add( 126  ,3100);
        RPMLUT.add( 130  ,3150);
        RPMLUT.add( 134  ,3175);
        RPMLUT.add( 137  ,3200);
        RPMLUT.add( 140  ,3225);
        RPMLUT.add( 143  ,3260);
        RPMLUT.add( 147  ,3300);
        RPMLUT.add( 150  ,3350);
        RPMLUT.add(maxDistance, 3350);
    }

    private void addAngleData(){
        AngleLUT.add(minDistance, 0.46);
        AngleLUT.add( 27.5 ,0.52); //.46
        AngleLUT.add( 45.6 ,0.6);
        AngleLUT.add( 47.2 ,0.65);
        AngleLUT.add( 54.6  ,0.7);
        AngleLUT.add( 60  ,0.72);
        AngleLUT.add( 66.1  ,0.78);
        AngleLUT.add( 77.1  ,0.8);
        AngleLUT.add( 90  ,0.81);
        //Cutoff 100 for close/far
        AngleLUT.add( 115  ,0.93);
        AngleLUT.add( 120  ,0.94);
        AngleLUT.add( 126  ,0.95);
        AngleLUT.add( 137  ,0.97);
        AngleLUT.add(maxDistance, 0.97);
    }

    private void addShotTimeData() {
        // TurtleWalkers moving-shot LUT (distance inches -> flight time seconds).
        // Thank you TurtleWalkers you guys are awesome
        ShotTimeLUT.add(0.0, 0.4); //.25
        ShotTimeLUT.add(42.5, 0.35); //.25
        ShotTimeLUT.add(55.0, 0.35); //.25
        ShotTimeLUT.add(66.7, 0.30);
        ShotTimeLUT.add(81.9, 0.50);
        ShotTimeLUT.add(95.7, 0.70);
        ShotTimeLUT.add(101.9, 0.70);
        ShotTimeLUT.add(116.6, 0.70);
        ShotTimeLUT.add(136.6, 0.80);
        ShotTimeLUT.add(3000.0, 1);
    }

    public double getRPMVal(double distance){
        if (distance < minDistance || distance > maxDistance){
            return -2;
        } else {
            return RPMLUT.get(distance);
        }
    }

    public double getAngleVal(double distance){
        if (distance < minDistance || distance > maxDistance){
            return -2;
        } else {
            return AngleLUT.get(distance);
        }
    }

    public double getShotTimeVal(double distance) {
        if (distance <= 0.0) {
            return ShotTimeLUT.get(0.0);
        }
        if (distance >= 3000.0) {
            return ShotTimeLUT.get(3000.0);
        }
        return ShotTimeLUT.get(distance);
    }

}
