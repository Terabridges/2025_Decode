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
import org.psilynx.psikit.core.Logger;
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
            speed = Math.pow(Math.abs(40 - kickerCurrentPos) / 120, 3);
            kickerLeft.setPower(speed/1.5);
            kickerRight.setPower(speed/1.5);

        } else if (pow < -0.2) {
            speed = Math.pow(Math.abs(350 - kickerCurrentPos) / 120, 3);
            kickerLeft.setPower(-speed/3);
            kickerRight.setPower(-speed/3);

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

    @Override
    public void logPsiKitData() {
        Logger.recordOutput("Subsystems/Other/Lift/EncoderDegree", kickerEnc.getCurrentPosition());
        Logger.recordOutput("Subsystems/Other/Lift/EncoderVoltage", kickerAnalog.getVoltage());
    }

}
