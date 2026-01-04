package org.firstinspires.ftc.teamcode.config.utility;

import com.arcrobotics.ftclib.util.InterpLUT;

public class ShooterData {

    public InterpLUT RPMLUT;
    public InterpLUT AngleLUT;
    public double minDistance = 0;
    public double maxDistance = 165;
    public double RPMMult = 0.85;

    public ShooterData(){
        RPMLUT = new InterpLUT();
        AngleLUT = new InterpLUT();
        addRPMData();
        addAngleData();
        RPMLUT.createLUT();
        AngleLUT.createLUT();
    }

    private void addRPMData(){
        RPMLUT.add(minDistance, 2900);
        RPMLUT.add( 22.8 ,2957);
        RPMLUT.add( 24.8 ,3041);
        RPMLUT.add( 33.6 ,3128);
        RPMLUT.add( 33.9 ,2858);
        RPMLUT.add( 34.05,3085);
        RPMLUT.add( 35.7 ,3100); //3200
        RPMLUT.add( 43   ,3290); //3385
        RPMLUT.add( 44.2 ,3310); //3385
        RPMLUT.add( 52.67,3600);
        RPMLUT.add( 53.7 ,3610); //3600
        RPMLUT.add( 61.4 ,3685);
        RPMLUT.add( 63.9 ,3685);
        RPMLUT.add( 67.4 ,3799); //3787
        RPMLUT.add( 72.47,3800); //4000
        RPMLUT.add( 72.5 ,3800); //4000
        RPMLUT.add( 83.7 ,3920); //4000
        RPMLUT.add( 122.5,4600);
        RPMLUT.add( 123.5,4600);
        RPMLUT.add( 126.7,4650);
        RPMLUT.add( 141  ,5000);
        RPMLUT.add(maxDistance, 5000);
    }

    private void addAngleData(){
        AngleLUT.add(minDistance, 0.2);
        AngleLUT.add( 22.8 ,0.2);
        AngleLUT.add( 24.8 ,0.225);
        AngleLUT.add( 33.6 ,0.3);
        AngleLUT.add( 33.9 ,0.275);
        AngleLUT.add( 34.05,0.25);
        AngleLUT.add( 35.7 ,0.2);
        AngleLUT.add( 43   ,0.4);
        AngleLUT.add( 44.2 ,0.4);
        AngleLUT.add( 52.67,0.5);
        AngleLUT.add( 53.7 ,0.5);
        AngleLUT.add( 61.4 ,0.5);
        AngleLUT.add( 63.9 ,0.525);
        AngleLUT.add( 67.4 ,0.55);
        AngleLUT.add( 72.47,0.55);
        AngleLUT.add( 72.5 ,0.55); //7
        AngleLUT.add( 83.7 ,0.6); //7
        AngleLUT.add( 122.5,0.7);
        AngleLUT.add( 123.5,0.7); //7
        AngleLUT.add( 126.7,0.7); //9 //8
        AngleLUT.add( 141  ,0.8); //8 //9
        AngleLUT.add(maxDistance, 0.8); //8 //9
    }

//    public double getRPMVal(double distance){
//        if (distance < minDistance || distance > maxDistance){
//            return -2;
//        } else if (distance < 30){
//            return RPMLUT.get(distance);
//        } else {
//            return RPMLUT.get(distance)*RPMMult;
//        }
//    }

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
}
