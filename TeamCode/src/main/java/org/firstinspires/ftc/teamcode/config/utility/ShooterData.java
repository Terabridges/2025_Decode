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
        RPMLUT.add(minDistance, 2225);
        RPMLUT.add( 37  ,2225);
        RPMLUT.add( 46  ,2300);
        RPMLUT.add( 52  ,2330);
        RPMLUT.add( 58  ,2355);
        RPMLUT.add( 62  ,2345);
        RPMLUT.add( 65  ,2475);
        RPMLUT.add( 66.5  ,2540);
        RPMLUT.add( 78  ,2630);
        RPMLUT.add( 91  ,2785);
        RPMLUT.add( 95  ,2795);
        RPMLUT.add( 101  ,2830);
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
        AngleLUT.add(minDistance, 0.4610);
        AngleLUT.add( 37  ,0.4610);
        AngleLUT.add( 46  ,0.5273);
        AngleLUT.add( 52  ,0.5914);
        AngleLUT.add( 58  ,0.6107);
        AngleLUT.add( 62  ,0.6382);
        AngleLUT.add( 65  ,0.6538);
        AngleLUT.add( 66.5  ,0.7152);
        AngleLUT.add( 78  ,0.7508);
        AngleLUT.add( 91  ,0.779);
        AngleLUT.add( 95  ,0.8288);
        AngleLUT.add( 101  ,0.8605);
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
