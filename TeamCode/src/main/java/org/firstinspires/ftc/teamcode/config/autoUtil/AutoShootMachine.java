package org.firstinspires.ftc.teamcode.config.autoUtil;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.config.subsystems.Robot;

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
        // TODO: wire this machine to V3 shooter/intake/vision commands.
        // TODO: each state below should call subsystem commands instead of placeholders.
        return new StateMachineBuilder()
                .state(ShootState.INIT)
                .transition(() -> startShooting, ShootState.PRESPIN)

                .state(ShootState.PRESPIN)
                .onEnter(() -> {
                    startShooting = false;
                    // TODO: start flywheel spin-up and intake feed prep.
                })
                .transitionTimed(spinUpTimeout, ShootState.CLUTCHDOWN)

                .state(ShootState.CLUTCHDOWN)
                .onEnter(() -> {
                    // TODO: clutch down for first shot.
                })
                .transitionTimed(clutchDownTime, ShootState.WAIT1)

                .state(ShootState.WAIT1)
                .transitionTimed(0.01, ShootState.SPIN)

                .state(ShootState.SPIN)
                .onEnter(() -> {
                    // TODO: index/feed first shot.
                })
                .transitionTimed(spinTime, ShootState.SPIN3)

                .state(ShootState.SPIN1)
                .onEnter(() -> {
                    // TODO: alternate far-shot feed step.
                })
                .transitionTimed(spinUpTimeout, ShootState.CLUTCHDOWN1)

                .state(ShootState.CLUTCHDOWN1)
                .onEnter(() -> {
                    // TODO: clutch sequencing between shot 1 and 2.
                })
                .transitionTimed(clutchDownFarTime, ShootState.SPIN2)
                .onExit(() -> {
                    // TODO: restore clutch hold between far-shot steps.
                })

                .state(ShootState.SPIN2)
                .onEnter(() -> {
                    // TODO: index/feed second shot.
                })
                .transitionTimed(spinUpTimeout, ShootState.CLUTCHDOWN2)

                .state(ShootState.CLUTCHDOWN2)
                .onEnter(() -> {
                    // TODO: clutch sequencing before final shot.
                })
                .transitionTimed(clutchDownFarTime, ShootState.SPIN3)
                .onExit(() -> {
                    // TODO: settle clutch before final feed.
                })

                .state(ShootState.SPIN3)
                .onEnter(() -> {
                    // TODO: index/feed final shot.
                })
                .transitionTimed(spinUpTimeout, ShootState.WAIT2)
                .onExit(() -> {
                    // TODO: clutch hold after final feed.
                })

                .state(ShootState.WAIT2)
                .transitionTimed(clutchDownFarTime, ShootState.INIT)
                .onExit(() -> {
                    // TODO: stop shooter/intake and reset feed/clutch state.
                    shootingComplete = true;
                })

                .build();
    }
}
