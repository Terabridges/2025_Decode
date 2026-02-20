package org.firstinspires.ftc.teamcode.config.utility;

import com.arcrobotics.ftclib.util.InterpLUT;

public class ShooterData {

    public InterpLUT RPMLUT;
    public InterpLUT AngleLUT;
    public double minDistance = 0;
    public double maxDistance = 165;

    public ShooterData(){
        RPMLUT = new InterpLUT();
        AngleLUT = new InterpLUT();
        addRPMData();
        addAngleData();
        RPMLUT.createLUT();
        AngleLUT.createLUT();
    }

    private void addRPMData(){
        RPMLUT.add(minDistance, 1500);
        RPMLUT.add( 27.5 ,2100);
        RPMLUT.add( 45.6 ,2200);
        RPMLUT.add( 47.2 ,2210);
        RPMLUT.add( 54.6  ,2250);
        RPMLUT.add( 60  ,2300);
        RPMLUT.add( 66.1  ,2360);
        RPMLUT.add( 77.1  ,2400);
        RPMLUT.add( 90  ,2550);
        RPMLUT.add( 128.6  ,3000);
        RPMLUT.add( 135.5  ,3100);
        RPMLUT.add(maxDistance, 3200);
    }

    private void addAngleData(){
        AngleLUT.add(minDistance, 0.46);
        AngleLUT.add( 27.5 ,0.46);
        AngleLUT.add( 45.6 ,0.6);
        AngleLUT.add( 47.2 ,0.65);
        AngleLUT.add( 54.6  ,0.7);
        AngleLUT.add( 60  ,0.72);
        AngleLUT.add( 66.1  ,0.78);
        AngleLUT.add( 77.1  ,0.8);
        AngleLUT.add( 90  ,0.81);
        AngleLUT.add( 128.6  ,0.91);
        AngleLUT.add( 135.5  ,0.92);
        AngleLUT.add(maxDistance, 0.93);
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

}
