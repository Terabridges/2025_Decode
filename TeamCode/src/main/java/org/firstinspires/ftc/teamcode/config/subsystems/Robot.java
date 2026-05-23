package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Drive;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Other.Other;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {
    private static final double METERS_TO_INCHES = 39.3701;

    public static class GoalTagRelocalizeResult {
        public final boolean success;
        public final int tagId;
        public final String reason;
        public final Pose followerPoseBefore;
        public final Pose relocalizedPose;

        public GoalTagRelocalizeResult(boolean success, int tagId, String reason, Pose followerPoseBefore, Pose relocalizedPose) {
            this.success = success;
            this.tagId = tagId;
            this.reason = reason;
            this.followerPoseBefore = followerPoseBefore;
            this.relocalizedPose = relocalizedPose;
        }
    }

    //---------------- Objects ----------------
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private Gamepad gp1;
    private Gamepad gp2;
    private static VoltageSensor voltageSensor;

    public enum ShootAllStates {
        INIT,
        GO_TO_SHOOT_ONE,
        WAIT0,
        WAIT1,
        GO_TO_SHOOT_TWO,
        WAIT2,
        GO_TO_SHOOT_THREE,
        WAIT3,
        RESET,
        UNJAM
    }

    public enum SortedShootAllStates {
        INIT,
        GO_TO_FIRST,
        WAIT0,
        WAIT1,
        GO_TO_SECOND,
        WAIT2,
        GO_TO_THIRD,
        WAIT3,
        RESET,
        UNJAM

    }

    public boolean initShootAllMachine = false;
    public boolean forceShootAllThreeOnNextStart = false;
    public boolean useAvailableBallCountForShootAll = false;

    public boolean initSortedShootAllMachine = false;

    private double waitTime = 0.01; //0.2

    private boolean goToReset = false;
    private int shootAllBallTargetCount = 0;
    public int sortedStartBall = 1;

    public boolean useSorting = true; //change to true
    private boolean wasFullLastLoop = false;
    public boolean txLights = false;


    //---------------- Subsystems ----------------

    public Intake intake;
    public Outtake outtake;
    public Other other;

    public List<org.firstinspires.ftc.teamcode.config.subsystems.Subsystem> subsystems;

    //---------------- Constructors ----------------
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gp1, Gamepad gp2){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        other = new Other(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(intake, outtake, other));

        this.gp1 = gp1;
        this.gp2 = gp2;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap, telemetry, null, null);
    }

    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

    public double getCurrentAmps() {
        if (intake == null) {
            return Double.NaN;
        }
        return intake.getFloodgateCurrentAmps();
    }

    public double getShootAllWaitTime() {
        return waitTime;
    }

    public boolean isGoToResetPending() {
        return goToReset;
    }

    public void toggleLightsTurret(){
        txLights = !txLights;
        if (!txLights){
            intake.lights.setFrontLight("clear");
            intake.lights.setMiddleLight("clear");
            intake.lights.setBackLight("clear");
        }
    }

//    public StateMachine getShootAllMachine(){
//        return new StateMachineBuilder()
//                .state(ShootAllStates.INIT)
//                .transition(() -> initShootAllMachine, ShootAllStates.GO_TO_SHOOT_ONE)
//                .onExit(()-> {
//                    initShootAllMachine = false;
//                    intake.spinner.setMegaSpinIn();
//                    outtake.shooter.useFlywheelPID = true;
//                    intake.spindex.setSpindexShootOnePre();
//                    intake.clutch.setClutchUp();
//                    outtake.shooter.setHoodTarget();
//                    intake.autoIntake = false;
//                })
//
//                .state(ShootAllStates.GO_TO_SHOOT_ONE)
//                .transition(()-> intake.spindex.isSpindexAtPos() && outtake.shooter.isAtRPM(), ShootAllStates.WAIT0)
//                .transition(()-> other.unJam, ShootAllStates.UNJAM)
//                .onExit(()-> {
//                    intake.clutch.setClutchDown();
//                    intake.spindex.setSpindexShootOne();
//                })
//
//                .state(ShootAllStates.WAIT0)
//                .transition(() -> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT1)
//                .transition(()-> other.unJam, ShootAllStates.UNJAM)
//
//                .state(ShootAllStates.WAIT1)
//                .transitionTimed(waitTime, ShootAllStates.GO_TO_SHOOT_TWO)
//                .transition(()-> other.unJam, ShootAllStates.UNJAM)
//
//                .state(ShootAllStates.GO_TO_SHOOT_TWO)
//                .onEnter(() -> {
//                    intake.spindex.setSpindexShootTwo();
//                })
//                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT2)
//                .transition(()-> other.unJam, ShootAllStates.UNJAM)
//
//                .state(ShootAllStates.WAIT2)
//                .transitionTimed(waitTime, ShootAllStates.GO_TO_SHOOT_THREE)
//                .transition(()-> other.unJam, ShootAllStates.UNJAM)
//
//                .state(ShootAllStates.GO_TO_SHOOT_THREE)
//                .onEnter(() -> {
//                    intake.spindex.setSpindexShootThree();
//                })
//                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT3)
//                .transition(()-> other.unJam, ShootAllStates.UNJAM)
//
//                .state(ShootAllStates.WAIT3)
//                .transitionTimed(waitTime, ShootAllStates.RESET)
//                .transition(()-> other.unJam, ShootAllStates.UNJAM)
//                .onExit(()-> {
//                    intake.spindex.setSpindexShootOnePreWrap();
//                })
//
//                .state(ShootAllStates.RESET)
//                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.INIT)
//                .onExit(()-> {
//                    txLights = false;
//                    shootAllBallTargetCount = 0;
//                    intake.spindex.setSpindexForwardOne();
//                    intake.spinner.setMegaSpinZero();
//                    intake.clutch.setClutchUp();
//                    intake.spindex.emptyBalls();
//                    intake.autoIntake = true;
//                })
//
//                .state(ShootAllStates.UNJAM)
//                .onEnter(()->{
//                    txLights = false;
//                    other.unJam = false;
//                    shootAllBallTargetCount = 0;
//                    intake.spindex.setSpindexDegree(intake.spindex.getAbsolutePos());
//                    intake.spinner.setMegaSpinZero();
//                    intake.clutch.setClutchUp();
//                    intake.spindex.emptyBalls();
//                    goToReset = true;
//                    intake.autoIntake = true;
//                })
//                .transition(()-> goToReset, ShootAllStates.INIT)
//                .onExit(()->goToReset = false)
//
//                .build();
//    }

    public int getLoadedBallCount() {
        if (intake == null || intake.spindex == null || intake.spindex.ballList == null) {
            return 0;
        }
        int count = 0;
        String[] ballList = intake.spindex.ballList;
        for (String slot : ballList) {
            if (slot != null && !slot.equals("E")) {
                count++;
            }
        }
        return count;
    }

    public StateMachine getSortedShootAllMachine(){
        return new StateMachineBuilder()
                .state(SortedShootAllStates.INIT)
                .transition(()-> initSortedShootAllMachine, SortedShootAllStates.GO_TO_FIRST)
                .onExit(()-> {
                    initSortedShootAllMachine = false;
                    outtake.setFastShootAllActive(false);
                    outtake.shooter.useFlywheelPID = true;
                    intake.clutch.setClutchUp();
                    intake.clutch.spinClutchIn();
                    outtake.shooter.setHoodTarget();
                    intake.autoIntake = false;
                    sortedStartBall = getBallSortedShootOrder();

                    if(sortedStartBall == 1){
                        intake.spindex.setSpindexShootOnePre();
                    } else if(sortedStartBall == 2){
                        intake.spindex.setSpindexShootTwoPre();
                    } else if(sortedStartBall == 3){
                        intake.spindex.setSpindexShootThreePre();
                    }
                })

                .state(SortedShootAllStates.GO_TO_FIRST)
                .transition(()-> intake.spindex.isSpindexAtPos() && outtake.shooter.isAtRPM(), SortedShootAllStates.WAIT0)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {
                    intake.clutch.setClutchDown();

                    if(sortedStartBall == 1){
                        intake.spindex.setSpindexShootOne();
                    } else if(sortedStartBall == 2){
                        intake.spindex.setSpindexShootTwo();
                    } else if(sortedStartBall == 3){
                        intake.spindex.setSpindexShootThree();
                    }
                })

                .state(SortedShootAllStates.WAIT0)
                .transition(()-> intake.spindex.isSpindexAtPos(), SortedShootAllStates.WAIT1)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)

                .state(SortedShootAllStates.WAIT1)
                .transitionTimed(waitTime, SortedShootAllStates.GO_TO_SECOND)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {

                    if(sortedStartBall == 1){
                        intake.spindex.setSpindexShootTwo();
                    } else if(sortedStartBall == 2){
                        intake.spindex.setSpindexShootThree();
                    } else if(sortedStartBall == 3){
                        intake.spindex.setSpindexShootOneWrap();
                    }
                })

                .state(SortedShootAllStates.GO_TO_SECOND)
                .transition(()-> intake.spindex.isSpindexAtPos(), SortedShootAllStates.WAIT2)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)

                .state(SortedShootAllStates.WAIT2)
                .transitionTimed(waitTime, SortedShootAllStates.GO_TO_THIRD)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {

                    if(sortedStartBall == 1){
                        intake.spindex.setSpindexShootThree();
                    } else if(sortedStartBall == 2){
                        intake.spindex.setSpindexShootOneWrap();
                    } else if(sortedStartBall == 3){
                        intake.spindex.setSpindexShootTwoWrap();
                    }
                })

                .state(SortedShootAllStates.GO_TO_THIRD)
                .transition(()-> intake.spindex.isSpindexAtPos(), SortedShootAllStates.WAIT3)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)

                .state(SortedShootAllStates.WAIT3)
                .transitionTimed(waitTime, SortedShootAllStates.RESET)
                .transition(()-> other.unJam, SortedShootAllStates.UNJAM)
                .onExit(()-> {
                    if(sortedStartBall == 1){
                        intake.spindex.setSpindexShootOnePreWrap();
                    } else if(sortedStartBall == 2){
                        intake.spindex.setSpindexShootTwoPreWrap();
                    } else if(sortedStartBall == 3){
                        intake.spindex.setSpindexShootThreePreWrap();
                    }
                })

                .state(SortedShootAllStates.RESET)
                .transition(()-> intake.spindex.isSpindexAtPos(), SortedShootAllStates.INIT)
                .onExit(()-> {
                    txLights = false;
                    intake.spindex.setSpindexForwardOne();
                    intake.clutch.setClutchUp();
                    intake.spindex.emptyBalls();
                    intake.clutch.spinClutchStop();
                    intake.autoIntake = true;
                })

                .state(SortedShootAllStates.UNJAM)
                .onEnter(()->{
                    txLights = false;
                    outtake.setFastShootAllActive(false);
                    other.unJam = false;
                    intake.spindex.setSpindexDegree(intake.spindex.getAbsolutePos());
                    intake.spinner.setMegaSpinZero();
                    intake.clutch.spinClutchStop();
                    intake.clutch.setClutchUp();
                    intake.spindex.emptyBalls();
                    intake.autoIntake = true;
                    goToReset = true;
                })
                .transition(()-> goToReset, SortedShootAllStates.INIT)
                .onExit(()->goToReset = false)

                .build();
    }

    public StateMachine getShootAllMachine(){
        return new StateMachineBuilder()
                .state(ShootAllStates.INIT)
                .transition(()-> initShootAllMachine, ShootAllStates.GO_TO_SHOOT_ONE)
                .onExit(()-> {
                    initShootAllMachine = false;
                    outtake.setFastShootAllActive(true);
                    outtake.shooter.useFlywheelPID = true;
                    intake.spindex.setSpindexShootOnePre();
                    intake.clutch.setClutchUp();
                    outtake.shooter.setHoodTarget();
                    intake.autoIntake = false;
                    intake.clutch.spinClutchIn();
                })

                .state(ShootAllStates.GO_TO_SHOOT_ONE)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.WAIT0)
                .onExit(()-> {
                    intake.clutch.setClutchDown();
                    intake.spindex.setSpindexShootThree();
                })

                .state(ShootAllStates.WAIT0)
                .transitionTimed(0.1, ShootAllStates.RESET)

                .state(ShootAllStates.RESET)
                .transition(()-> intake.spindex.isSpindexAtPos(), ShootAllStates.INIT)
                .onExit(()-> {
                    txLights = false;
                    outtake.setFastShootAllActive(false);
                    shootAllBallTargetCount = 0;
                    intake.spindex.setSpindexForwardOne();
                    intake.clutch.setClutchUp();
                    intake.clutch.spinClutchStop();
                    intake.spindex.emptyBalls();
                    intake.autoIntake = true;
                })

                .build();

    }

    public int getBallSortedShootOrder(){
        if(GlobalVariables.getMotif().equals(GlobalVariables.MotifPattern.PPG)) {
            if (intake.spindex.balls.equals("PPG")){
                return 1;
            } else if (intake.spindex.balls.equals("GPP")){
                return 2;
            } else if (intake.spindex.balls.equals("PGP")){
                return 3;
            } else {
                return 1;
            }
        } else if(GlobalVariables.getMotif().equals(GlobalVariables.MotifPattern.GPP)) {
            if (intake.spindex.balls.equals("GPP")){
                return 1;
            } else if (intake.spindex.balls.equals("PGP")){
                return 2;
            } else if (intake.spindex.balls.equals("PPG")){
                return 3;
            } else {
                return 1;
            }
        } else if(GlobalVariables.getMotif().equals(GlobalVariables.MotifPattern.PGP)) {
            if (intake.spindex.balls.equals("PGP")){
                return 1;
            } else if (intake.spindex.balls.equals("PPG")){
                return 2;
            } else if (intake.spindex.balls.equals("GPP")){
                return 3;
            } else {
                return 1;
            }
        } else {
            return 1;
        }
    }

    public void getReadyShoot(){
        if (useSorting){
            int firstBall = getBallSortedShootOrder();
            if (firstBall == 1){
                intake.spindex.setSpindexShootOnePre();
            } else if (firstBall == 2){
                intake.spindex.setSpindexShootTwoPre();
            } else if (firstBall == 3) {
                intake.spindex.setSpindexShootThreePre();
            }
        } else {
            intake.spindex.setSpindexShootOnePre();
        }
        outtake.shooter.setHoodTarget();
    }

    public void toggleSorting(){
        useSorting = !useSorting;
        intake.useSortingIntake = useSorting;
        if (intake.useSortingIntake == false){
            intake.lights.setSortingLights();
        }
    }

    /**
     * Points turret forward, then relocalizes follower from visible goal AprilTag (20 or 24).
     * Call repeatedly in a loop while aiming; returns success once a valid goal-tag botpose is available.
     */
    public GoalTagRelocalizeResult relocalizeFromGoalTag() {
        return relocalizeFromGoalTag(Turret.turretForwardDeg);
    }

    /**
     * Same as {@link #relocalizeFromGoalTag()} but with caller-specified forward turret heading.
     */
    public GoalTagRelocalizeResult relocalizeFromGoalTag(double forwardTurretDeg) {
        outtake.turret.setTurretDegree(forwardTurretDeg);

        Pose before = snapshotPose(follower != null ? follower.getPose() : null);
        if (follower == null) {
            return new GoalTagRelocalizeResult(false, -1, "Follower not initialized", before, null);
        }

        int tagId = outtake.vision.getVisibleGoalTagId();
        if (tagId < 0) {
            return new GoalTagRelocalizeResult(false, -1, "No goal tag (20/24) visible", before, null);
        }

        Pose3D llPose = outtake.vision.getLatestBotPose();
        if (llPose == null) {
            return new GoalTagRelocalizeResult(false, tagId, "Goal tag visible but no Limelight botpose", before, null);
        }

        double[] compensated = outtake.vision.getTurretCompensatedPose2dMetersFromPose3d(llPose);
        double xMeters = llPose.getPosition().x;
        double yMeters = llPose.getPosition().y;
        double headingDeg = llPose.getOrientation().getYaw(AngleUnit.DEGREES);
        if (compensated != null && compensated.length >= 3) {
            xMeters = compensated[0];
            yMeters = compensated[1];
            headingDeg = compensated[2];
        }

        double xIn = xMeters * METERS_TO_INCHES;
        double yIn = yMeters * METERS_TO_INCHES;
        double headingRad = Math.toRadians(headingDeg);
        Pose relocalized = new Pose(xIn, yIn, headingRad);
        follower.setPose(relocalized);
        return new GoalTagRelocalizeResult(true, tagId, "Relocalized from goal tag", before, snapshotPose(relocalized));
    }

    private Pose snapshotPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    //---------------- Interface Methods ----------------
    public void update() {
        for (org.firstinspires.ftc.teamcode.config.subsystems.Subsystem s : subsystems) {
            s.updateWithTiming();
            s.logPsiKitData();
        }

        boolean isFull = intake.spindex.loadedBallCount() == 3;
        if (isFull && !wasFullLastLoop) {
            getReadyShoot();
        }
        wasFullLastLoop = isFull;

        if (txLights){
            if(outtake.vision.getTx()<2){
                intake.lights.setFrontLight("yellow");
                intake.lights.setMiddleLight("yellow");
                intake.lights.setBackLight("yellow");
            } else {
                intake.lights.setFrontLight("red");
                intake.lights.setMiddleLight("red");
                intake.lights.setBackLight("red");
            }
        }
    }

    public void toInit() {
        for (Subsystem s : subsystems) {
            s.toInit();
        }
    }
}
