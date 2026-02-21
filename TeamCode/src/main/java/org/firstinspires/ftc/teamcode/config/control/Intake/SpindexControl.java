package org.firstinspires.ftc.teamcode.config.control.Intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Spindex;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

@Configurable
public class SpindexControl implements Control {

    //---------------- Software ----------------
    Spindex spindex;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;
    EdgeDetector ballClockwise = new EdgeDetector(()-> spindex.moveBallClockwise());
    EdgeDetector ballCounter = new EdgeDetector(()-> spindex.moveBallCounter());
    EdgeDetector switchDirection = new EdgeDetector(()-> spindex.switchSides());
    //EdgeDetector toggleShootMode = new EdgeDetector(()-> spindex.toggleShootMode());
    EdgeDetector emptyBalls = new EdgeDetector(()-> spindex.emptyBalls());
    public static int fullSpindexRumbleMs = 500;
    private boolean wasFullLastLoop = false;


    //---------------- Constructor ----------------
    public SpindexControl(Spindex spindex, Gamepad gp1, Gamepad gp2){
        this.spindex = spindex;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public SpindexControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.intake.spindex, gp1, gp2);
        this.robot = robot;
    }

    //---------------- Methods ----------------
    private int loadedBallCount() {
        int count = 0;
        if (spindex.ballList[0] != null && !spindex.ballList[0].equals("E")) count++;
        if (spindex.ballList[1] != null && !spindex.ballList[1].equals("E")) count++;
        if (spindex.ballList[2] != null && !spindex.ballList[2].equals("E")) count++;
        return count;
    }


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        ballClockwise.update(gp1.dpad_right);
        ballCounter.update(gp1.dpad_left);
        switchDirection.update(gp1.dpad_up);
        //toggleShootMode.update(gp1.back);
        emptyBalls.update(gp1.right_stick_button);

        boolean isFull = loadedBallCount() == 3;
        if (isFull && !wasFullLastLoop) {
            gp1.rumble(Math.max(0, fullSpindexRumbleMs));
        }
        wasFullLastLoop = isFull;

    }

    @Override
    public void addTelemetry(Telemetry telemetry){
        telemetry.addData("Current Direction", spindex.getCurrentDirection());
        telemetry.addData("Current Ball", spindex.getCurrentBall());
        telemetry.addData("CurrentPos", spindex.getAbsolutePos());
        telemetry.addData("CommandedPos", spindex.getCommandedPos());
        telemetry.addData("Ball Order", spindex.balls);
        telemetry.addData("FrontRed", spindex.frontRed);
        telemetry.addData("FrontGreen", spindex.frontGreen);
        telemetry.addData("FrontBlue", spindex.frontBlue);
        telemetry.addData("BackRed", spindex.backRed);
        telemetry.addData("BackGreen", spindex.backGreen);
        telemetry.addData("BackBlue", spindex.backBlue);
        telemetry.addData("Spindex Full", wasFullLastLoop);
    }
}
