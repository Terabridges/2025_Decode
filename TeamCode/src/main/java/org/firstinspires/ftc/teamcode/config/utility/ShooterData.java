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
        RPMLUT.add(minDistance, 3700);
        RPMLUT.add(22.38, 3600);
        RPMLUT.add(40, 3700);
        RPMLUT.add(61.5, 3800);
        RPMLUT.add(74.4, 4000);
        RPMLUT.add(88, 4500);
        RPMLUT.add(126.5, 5700);
        RPMLUT.add(145, 5700);
        RPMLUT.add(maxDistance, 5700);
    }

    private void addAngleData(){
        AngleLUT.add(minDistance, 0);
        AngleLUT.add(22.38, 0.25);
        AngleLUT.add(40, 0.4);
        AngleLUT.add(61.5, 0.4);
        AngleLUT.add(74.4, 0.55);
        AngleLUT.add(88, 0.65);
        AngleLUT.add(126.5, 0.83);
        AngleLUT.add(145, 0.95);
        AngleLUT.add(maxDistance, 0.95);
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
