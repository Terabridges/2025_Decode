package org.firstinspires.ftc.teamcode.config.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class TransferControl implements Control {

    //---------------- Software ----------------
    Transfer transfer;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector transferLeftRE = new EdgeDetector( () -> transfer.spindexLeft());
    EdgeDetector transferLeftFE = new EdgeDetector( () -> transfer.spindexZero(), true);
    EdgeDetector transferRightRE = new EdgeDetector( () -> transfer.spindexRight());
    EdgeDetector transferRightFE = new EdgeDetector( () -> transfer.spindexZero(), true);
    EdgeDetector toggleClutch = new EdgeDetector(() -> transfer.toggleClutch());

    //---------------- Constructor ----------------
    public TransferControl(Transfer transfer, Gamepad gp1, Gamepad gp2){
        this.transfer = transfer;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public TransferControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.transfer, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        transferLeftRE.update(gp1.dpad_left);
        transferLeftFE.update(gp1.dpad_left);
        transferRightRE.update(gp1.dpad_right);
        transferRightFE.update(gp1.dpad_right);
        toggleClutch.update(gp1.b);

    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
