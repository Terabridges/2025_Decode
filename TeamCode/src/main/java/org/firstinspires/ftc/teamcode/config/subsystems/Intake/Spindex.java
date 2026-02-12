package org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;

public class Spindex implements Subsystem {

    //---------------- Hardware ----------------
    private CRServo spindexLeft;
    private CRServo spindexRight;
    private AnalogInput spindexAnalog;
    private AbsoluteAnalogEncoder spindexEnc;
    private RevColorSensorV3 frontColor;
    private RevColorSensorV3 middleColor;
    private RevColorSensorV3 backColor;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Spindex(HardwareMap map) {
        spindexLeft = map.get(CRServo.class, "spindexL");
        spindexRight = map.get(CRServo.class, "spindexR");
        spindexLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontColor = map.get(RevColorSensorV3.class, "color1");
        middleColor = map.get(RevColorSensorV3.class, "color2");
        backColor = map.get(RevColorSensorV3.class, "color3");
        spindexAnalog = map.get(AnalogInput.class, "spindexAnalog");
        spindexEnc = new AbsoluteAnalogEncoder(spindexAnalog, 3.3, 0, 1);
    }

    //---------------- Methods ----------------
    public void moveSpindexPow(double pow){
        spindexLeft.setPower(pow);
        spindexRight.setPower(pow);
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){

    }

}
