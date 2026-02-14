package org.firstinspires.ftc.teamcode.config.autoUtil;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;

public class AutoShootMachine {

    public enum ShootState {
        INIT,
        PRESPIN,
        CLUTCHDOWN,
        WAIT1,
        SPIN,
        SPIN1,
        CLUTCHDOWN1,
        SPIN2,
        CLUTCHDOWN2,
        SPIN3,
        WAIT2
    }

    private final Robot robot;
    private final double clutchDownTime;
    private final double clutchDownFarTime;
    private final double spinTime;
    private final double spinUpTimeout;

    private final StateMachine machine;
    private boolean startShooting = false;
    private boolean shootingComplete = false;

    public AutoShootMachine(Robot robot,
                            double clutchDownTime,
                            double clutchDownFarTime,
                            double spinTime,
                            double spinUpTimeout) {
        this.robot = robot;
        this.clutchDownTime = clutchDownTime;
        this.clutchDownFarTime = clutchDownFarTime;
        this.spinTime = spinTime;
        this.spinUpTimeout = spinUpTimeout;
        this.machine = buildMachine();
    }

    public void start() {
        machine.start();
    }

    public void update() {
        machine.update();
    }

    public ShootState getState() {
        return (ShootState) machine.getState();
    }

    public void requestShoot() {
        startShooting = true;
    }

    public boolean isShootingComplete() {
        return shootingComplete;
    }

    public void clearShootingComplete() {
        shootingComplete = false;
    }

    private StateMachine buildMachine() {
        Shooter shooter = robot.shooter;
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(ShootState.INIT)
                .transition(() -> (startShooting && shooter.hasDesiredTarget && Math.abs(robot.vision.getTx()) < 6), ShootState.PRESPIN)

                .state(ShootState.PRESPIN)
                .onEnter(() -> {
                    startShooting = false;
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    shooter.shooterShoot = true;
                    transfer.isDetecting = false;
                    if (transfer.desiredRotate == 1) {
                        transfer.ballLeft();
                    } else if (transfer.desiredRotate == 2) {
                        transfer.ballRight();
                    }
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), ShootState.CLUTCHDOWN)
                .transitionTimed(spinUpTimeout, ShootState.CLUTCHDOWN)

                .state(ShootState.CLUTCHDOWN)
                .onEnter(() -> {
                    transfer.max = 0.275;
                    transfer.setClutchBarelyDown();
                })
                .transitionTimed(clutchDownTime, ShootState.WAIT1)

                .state(ShootState.WAIT1)
                .transition(() -> shooter.isFarShot(), ShootState.SPIN1)
                .transition(() -> !shooter.isFarShot(), ShootState.SPIN)

                .state(ShootState.SPIN)
                .onEnter(() -> {
                    transfer.max = 0.2;
                    transfer.ballLeft();
                    transfer.ballLeft();
                })
                .transition(() -> transfer.spindexAtTarget(), ShootState.SPIN3)
                .transitionTimed(spinTime, ShootState.SPIN3)

                .state(ShootState.SPIN1)
                .onEnter(() -> {
                    transfer.ballLeftSmall();
                    shooter.hoodOffset -= 0.06;
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), ShootState.CLUTCHDOWN1)
                .transitionTimed(spinUpTimeout, ShootState.CLUTCHDOWN1)

                .state(ShootState.CLUTCHDOWN1)
                .onEnter(() -> {
                    transfer.setClutchDownFar();
                    shooter.useTurretPID = false;
                })
                .transitionTimed(clutchDownFarTime, ShootState.SPIN2)
                .onExit(() -> transfer.setClutchBarelyDown())

                .state(ShootState.SPIN2)
                .onEnter(() -> {
                    transfer.ballLeft();
                    shooter.hoodOffset += 0.06;
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), ShootState.CLUTCHDOWN2)
                .transitionTimed(spinUpTimeout, ShootState.CLUTCHDOWN2)

                .state(ShootState.CLUTCHDOWN2)
                .onEnter(() -> transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, ShootState.SPIN3)
                .onExit(() -> {
                    transfer.setClutchBarelyDown();
                    transfer.ballRightSmall();
                })

                .state(ShootState.SPIN3)
                .onEnter(() -> {
                    transfer.max = 0.275;
                    transfer.ballLeftSmall();
                    transfer.ballLeft();
                })
                .transition(() -> transfer.spindexAtTarget() && shooter.isAtRPM(), ShootState.WAIT2)
                .transitionTimed(spinUpTimeout, ShootState.WAIT2)
                .onExit(() -> transfer.setClutchDownFar())

                .state(ShootState.WAIT2)
                .transitionTimed(clutchDownFarTime, ShootState.INIT)
                .onExit(() -> {
                    transfer.setClutchUp();
                    transfer.ballRightSmall();
                    intake.spinnerMacroTarget = 0;
                    shooter.shooterShoot = false;
                    transfer.isDetecting = true;
                    transfer.emptyBalls();
                    intake.spinnerMacro = false;
                    transfer.max = 0.4;
                    shootingComplete = true;
                    shooter.useTurretPID = true;
                })

                .build();
    }
}
