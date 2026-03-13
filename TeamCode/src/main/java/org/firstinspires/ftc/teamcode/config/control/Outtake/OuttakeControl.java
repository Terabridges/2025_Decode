package org.firstinspires.ftc.teamcode.config.control.Outtake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class OuttakeControl implements Control {

    //---------------- Software ----------------
    Outtake outtake;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector increaseOffset = new EdgeDetector(()-> outtake.increaseOffset());
    EdgeDetector decreaseOffset = new EdgeDetector(()-> outtake.decreaseOffset());
    EdgeDetector changeOffset = new EdgeDetector(()-> outtake.changeOffsetType());
    EdgeDetector resetOffset = new EdgeDetector(()-> outtake.resetOffset());


    //---------------- Constructor ----------------
    public OuttakeControl(Outtake outtake, Gamepad gp1, Gamepad gp2){
        this.outtake = outtake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public OuttakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.outtake, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        increaseOffset.update(gp2.right_bumper);
        decreaseOffset.update(gp2.left_bumper);
        changeOffset.update(gp2.a);
        resetOffset.update(gp2.start);
    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        //telemetry.addData("Recoil Comp Enabled", Outtake.enableRpmRecoilComp);
        //telemetry.addData("Recoil RPM Error", "%.1f", outtake.getLastRecoilRpmError());
        //telemetry.addData("Recoil Hood Delta", "%.4f", outtake.getLastRecoilHoodDelta());
        //telemetry.addData("Hood Base/Comp", "%.4f / %.4f", outtake.getLastBaseHoodPos(), outtake.getLastCompedHoodPos());

        telemetry.addData("Offset Type", outtake.currentOffsetType);
        telemetry.addData("Heading Trim Offset", outtake.turretAimTrimOffsetDeg);
        telemetry.addData("Vision B Offset", outtake.turretAimCommandOffsetDeg);
        telemetry.addData("Total Heading Offset", Outtake.getTotalTurretAimCommandOffsetDeg());
        telemetry.addData("RPM Type", outtake.shooter.flywheelOffset);
        telemetry.addData("Hood Type", outtake.shooter.hoodOffset);
    }
}
