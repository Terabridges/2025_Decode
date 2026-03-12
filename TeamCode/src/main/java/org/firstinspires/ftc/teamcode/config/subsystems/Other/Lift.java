package org.firstinspires.ftc.teamcode.config.subsystems.Other;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.config.utility.AbsoluteAnalogEncoder;
import org.psilynx.psikit.ftc.autolog.PsiKitFieldAutoLog;

@PsiKitFieldAutoLog
public class Lift implements Subsystem {

    //---------------- Hardware ----------------
    private CRServo kickerLeft;
    private CRServo kickerRight;
    private AnalogInput kickerAnalog;
    private AbsoluteAnalogEncoder kickerEnc;

    //---------------- Software ----------------
    //355 up, goes down until 10
    private double upLimit = 345;
    private double downLimit = 20;
    public double speed;
    public double kickerCurrentPos = 330;
    //also, note that gp y is inverted; pushing up yields -1

    //---------------- Constructor ----------------
    public Lift(HardwareMap map) {
        kickerLeft = map.get(CRServo.class, "kickL");
        kickerRight = map.get(CRServo.class, "kickR");
        kickerAnalog = map.get(AnalogInput.class, "kickAnalog");
        kickerEnc = new AbsoluteAnalogEncoder(kickerAnalog, 3.3, 86, 1);
        kickerLeft.setDirection(CRServo.Direction.FORWARD);
        kickerRight.setDirection(CRServo.Direction.REVERSE);
    }

    //---------------- Methods ----------------
    //pow greater than one moves spindex down, aka using lift
    public void moveKicker(double pow){
        if (pow > 0.2) {
            speed = Math.min(1.0, Math.pow(Math.abs(10 - kickerCurrentPos) / 160.0, 4));
            kickerLeft.setPower(speed/4);
            kickerRight.setPower(speed/4);

        } else if (pow < -0.2) {
            speed = Math.min(1.0, Math.pow(Math.abs(355 - kickerCurrentPos) / 160.0, 4));
            kickerLeft.setPower(-speed/4);
            kickerRight.setPower(-speed/4);

        } else {
            kickerLeft.setPower(0);
            kickerRight.setPower(0);
        }
    }

    //---------------- Interface Methods ----------------
    @Override
    public void toInit(){

    }

    @Override
    public void update(){
        kickerCurrentPos = kickerEnc.getCurrentPosition();

    }
}
