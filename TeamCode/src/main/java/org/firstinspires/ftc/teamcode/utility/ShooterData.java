package org.firstinspires.ftc.teamcode.utility;

import com.arcrobotics.ftclib.util.InterpLUT;

public class ShooterData {

    public InterpLUT RPMLUT;
    public InterpLUT AngleLUT;

    public ShooterData(){
        RPMLUT = new InterpLUT();
        AngleLUT = new InterpLUT();
        addRPMData();
        addAngleData();
        RPMLUT.createLUT();
        AngleLUT.createLUT();
    }

    private void addRPMData(){
        RPMLUT.add(23.5, 3100);
        RPMLUT.add(29.5, 3100);
        RPMLUT.add(39.5, 3250);
        RPMLUT.add(59.9, 3700);
        RPMLUT.add(68.8, 3700);
        RPMLUT.add(79.5, 4000);
    }

    private void addAngleData(){
        AngleLUT.add(23.5, 0.175);
        AngleLUT.add(29.5, 0.35);
        AngleLUT.add(39.5, 0.475);
        AngleLUT.add(59.9, 0.8);
        AngleLUT.add(68.8, 0.8);
        AngleLUT.add(79.5, 0.8);
    }

    public double getRPMVal(double distance){
        return RPMLUT.get(distance);
    }

    public double getAngleVal(double distance){
        return AngleLUT.get(distance);
    }
}
