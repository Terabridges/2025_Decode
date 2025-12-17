package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.config.control.Control;
import org.firstinspires.ftc.teamcode.config.control.DriveControl;
import org.firstinspires.ftc.teamcode.config.control.IntakeControl;
import org.firstinspires.ftc.teamcode.config.control.ShooterControl;
import org.firstinspires.ftc.teamcode.config.control.TransferControl;
import org.firstinspires.ftc.teamcode.config.control.VisionControl;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;
import org.firstinspires.ftc.teamcode.logging.PsiKitDriverStationLogger;
import org.firstinspires.ftc.teamcode.logging.PsiKitMotorLogger;
import org.firstinspires.ftc.teamcode.logging.PsiKitPinpointV2Logger;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.ftc.PsiKitLinearOpMode;
import org.psilynx.psikit.ftc.PsiKitOpMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="MainTeleOpLogging", group="TeleOp")
public class MainTeleopLogging extends PsiKitLinearOpMode {

    private final PsiKitDriverStationLogger driverStationLogger = new PsiKitDriverStationLogger();
    private final PsiKitMotorLogger motorLogger = new PsiKitMotorLogger();
    private final PsiKitPinpointV2Logger pinpointLogger = new PsiKitPinpointV2Logger();

    public DriveControl driveControl;
    public IntakeControl intakeControl;
    public ShooterControl shooterControl;
    public TransferControl transferControl;
    public VisionControl visionControl;
    public List<Control> controls;

    public Gamepad currentGamepad1;
    public Gamepad previousGamepad1;

    public Gamepad currentGamepad2;
    public Gamepad previousGamepad2;

//    public enum shootStates {
//        INIT,
//        SPIN0,
//        CLUTCHDOWN1,
//        CLUTCHDOWNFAR1,
//        SPIN1,
//        CLUTCHDOWN2,
//        CLUTCHDOWNFAR2,
//        SPIN2,
//        CLUTCHDOWN3,
//        CLUTCHDOWNFAR3,
//    }

    public enum shootStates {
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

    public enum clutchStates {
        INIT,
        SPIN,
        CLUTCHDOWN,
        CLUTCHUP
    }

    public enum resetStates {
        INIT,
        SPIN,
        RESET,
        SPIN2,
        RESET2
    }

    public double clutchDownTime = 0.1;
    public double clutchDownFarTime = 0.4;
    public double spinTime = 2.75;
    public double spinUpTimeout = 1.75;
    int fromRed = -90;

    public StateMachine shootAllMachine;
    public StateMachine clutchSuperMachine;
    public StateMachine resetMachine;

    @Override
    public void runOpMode(){
        // If the prior OpMode was force-stopped, PsiKit may still be "running".
        // Clean up so each run produces a fresh file and frees port 5800.
        try { Logger.end(); } catch (Exception ignored) {}
        Logger.reset();

        psiKitSetup();
        Logger.addDataReceiver(new RLOGServer());
        String filename = this.getClass().getSimpleName() + "_log_" + new java.text.SimpleDateFormat("yyyyMMdd_HHmmss_SSS").format(new java.util.Date()) + ".rlog";
        Logger.addDataReceiver(new RLOGWriter(filename));
        Logger.recordMetadata("some metadata", "string value");
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        try {
            Robot robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

            driveControl = new DriveControl(robot, gamepad1, gamepad2);
            intakeControl = new IntakeControl(robot, gamepad1, gamepad2);
            shooterControl = new ShooterControl(robot, gamepad1, gamepad2);
            transferControl = new TransferControl(robot, gamepad1, gamepad2);
            visionControl = new VisionControl(robot, gamepad1, gamepad2);

            controls = new ArrayList<>(Arrays.asList(intakeControl, shooterControl, transferControl, driveControl, visionControl));

            currentGamepad1 = new Gamepad();
            previousGamepad1 = new Gamepad();

            currentGamepad2 = new Gamepad();
            previousGamepad2 = new Gamepad();

            shootAllMachine = getShootAllMachine(robot);
            clutchSuperMachine = getClutchSuperMachine(robot);
            resetMachine = getSpindexResetMachine(robot);

            robot.transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        while (!getPsiKitIsStarted()){
//            //processHardwareInputs();
//            Logger.periodicBeforeUser();
//
//            previousGamepad1.copy(currentGamepad1);
//            currentGamepad1.copy(gamepad1);
//
//            if (currentGamepad1.a && !previousGamepad1.a){
//                if(GlobalVariables.motif.equals("PPG")){
//                    GlobalVariables.motif = "GPP";
//                } else if(GlobalVariables.motif.equals("GPP")){
//                    GlobalVariables.motif = "PGP";
//                } else if(GlobalVariables.motif.equals("PGP")){
//                    GlobalVariables.motif = "PPG";
//                }
//            }
//
//            if (currentGamepad1.b && !previousGamepad1.b){
//                if(GlobalVariables.allianceColor.equals("red")){
//                    GlobalVariables.allianceColor = "blue";
//                } else if (GlobalVariables.allianceColor.equals("blue")){
//                    GlobalVariables.allianceColor = "red";
//                }
//            }
//
//            telemetry.addData("Press A to change Motif. Press B to change alliance color.", "");
//            telemetry.addData("Motif", GlobalVariables.motif);
//            telemetry.addData("Alliance Color", GlobalVariables.allianceColor);
//            telemetry.update();
//
//            Logger.periodicAfterUser(0.0, 0.0);
//        }

            waitForStart();

            robot.toInit();
            shootAllMachine.start();
            clutchSuperMachine.start();
            resetMachine.start();

            while (opModeIsActive()){
                double beforeUserStart = Logger.getTimestamp();
                Logger.periodicBeforeUser();
                processHardwareInputs();
                double beforeUserEnd = Logger.getTimestamp();

                driverStationLogger.log(gamepad1, gamepad2);
                motorLogger.logAll(hardwareMap);
                pinpointLogger.logAll(hardwareMap);

                gamepadUpdate();
                robot.update();
                controlsUpdate();
                stateMachinesUpdate();


            //Stop everything
            if(currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button){
                shootAllMachine.setState(shootStates.INIT);
                clutchSuperMachine.setState(clutchStates.INIT);
                resetMachine.setState(resetStates.INIT);
                robot.intake.spinnerMacroTarget = 0;
                robot.shooter.shooterShoot = false;
                robot.transfer.isDetecting = true;
                robot.transfer.emptyBalls();
                robot.intake.spinnerMacro = false;
                robot.transfer.max = 0.4;
            }

            //Switch Alliance
            if(currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                if(GlobalVariables.allianceColor.equals("red")){
                    GlobalVariables.allianceColor = "blue";
                } else if (GlobalVariables.allianceColor.equals("blue")){
                    GlobalVariables.allianceColor = "red";
                }
            }

            //Switch Motif
            if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper){
                if(GlobalVariables.motif.equals("PPG")){
                    GlobalVariables.motif = "GPP";
                } else if(GlobalVariables.motif.equals("GPP")){
                    GlobalVariables.motif = "PGP";
                } else if(GlobalVariables.motif.equals("PGP")){
                    GlobalVariables.motif = "PPG";
                }
            }

            Logger.recordOutput("OpMode/example", 2.0);
            Logger.recordOutput("spindexPos", robot.transfer.spindex.getCurrentPosition());
            Logger.recordOutput("visionError", robot.vision.getTx());
                double afterUserStart = Logger.getTimestamp();
                Logger.periodicAfterUser(
                        afterUserStart - beforeUserEnd,
                        beforeUserEnd - beforeUserStart
                );

                idle();
            }
        } finally {
            try { Logger.end(); } catch (Exception ignored) {}
        }
    }

    public void controlsUpdate() {
        for (Control c : controls) {
            c.update();
            c.addTelemetry(telemetry);
        }
        telemetry.addData("Shooting State", shootAllMachine.getState());
        telemetry.update();
    }

    public void gamepadUpdate(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }

    public void stateMachinesUpdate(){
        shootAllMachine.update();
        clutchSuperMachine.update();
        resetMachine.update();
    }

//    public StateMachine getShootAllMachine (Robot robot){
//        Shooter shooter = robot.shooter;
//        Transfer transfer = robot.transfer;
//        Intake intake = robot.intake;
//        return new StateMachineBuilder()
//                .state(shootStates.INIT)
//                .transition(()->(currentGamepad1.x && !previousGamepad1.x), shootStates.SPIN0)
//
//                .state(shootStates.SPIN0)
//                .onEnter(()-> {
//                    transfer.ballRightSmall();
//                    intake.spinnerMacro = true;
//                    intake.spinnerMacroTarget = 0.95;
//                    shooter.shooterShoot = true;
//                    transfer.isDetecting = false;
//                    if(transfer.desiredRotate == 3 || transfer.desiredRotate == 5){
//                        transfer.ballLeft();
//                    } else if (transfer.desiredRotate == 4){
//                        transfer.ballRight();
//                    }
//                })
//                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN1)
//                .transitionTimed(2.25, shootStates.CLUTCHDOWN1)
//
//                .state(shootStates.CLUTCHDOWN1)
//                .onEnter(()-> transfer.setClutchDown())
//                .transitionTimed(clutchDownTime, shootStates.CLUTCHDOWNFAR1)
//
//                .state(shootStates.CLUTCHDOWNFAR1)
//                .onEnter(()-> transfer.setClutchDownFar())
//                .transitionTimed(clutchDownFarTime, shootStates.SPIN1)
//                .onExit(()-> {
//                    transfer.setClutchUp();
//                })
//
//                .state(shootStates.SPIN1)
//                .onEnter(()-> {
//                    if (transfer.desiredRotate == 1 || transfer.desiredRotate == 3){
//                        transfer.ballLeft();
//                    } else if (transfer.desiredRotate == 2 || transfer.desiredRotate == 4 || transfer.desiredRotate == 5) {
//                        transfer.ballRight();
//                    }
//                })
//                .transition(()-> transfer.spindexAtTarget(), shootStates.CLUTCHDOWN2)
//                .transitionTimed(1.5, shootStates.CLUTCHDOWN2)
//
//                .state(shootStates.CLUTCHDOWN2)
//                .onEnter(()-> {
//                    transfer.setClutchDown();
//                })
//                .transitionTimed(clutchDownTime, shootStates.CLUTCHDOWNFAR2)
//
//                .state(shootStates.CLUTCHDOWNFAR2)
//                .onEnter(()-> transfer.setClutchDownFar())
//                .transitionTimed(clutchDownFarTime, shootStates.SPIN2)
//                .onExit(()-> transfer.setClutchUp())
//
//                .state(shootStates.SPIN2)
//                .onEnter(()-> {
//                    if (transfer.desiredRotate == 1 || transfer.desiredRotate == 3){
//                        transfer.ballLeft();
//                    } else if (transfer.desiredRotate == 2 || transfer.desiredRotate == 4 || transfer.desiredRotate == 5) {
//                        transfer.ballRight();
//                    }
//                })
//                .transition(()-> transfer.spindexAtTarget(), shootStates.CLUTCHDOWN3)
//                .transitionTimed(1.5, shootStates.CLUTCHDOWN3)
//
//                .state(shootStates.CLUTCHDOWN3)
//                .onEnter(()-> {
//                    transfer.setClutchDown();
//                })
//                .transitionTimed(clutchDownTime, shootStates.CLUTCHDOWNFAR3)
//
//                .state(shootStates.CLUTCHDOWNFAR3)
//                .onEnter(()-> transfer.setClutchDownFar())
//                .transitionTimed(clutchDownFarTime, shootStates.INIT)
//                .onExit(()-> {
//                    transfer.setClutchUp();
//                    intake.spinnerMacroTarget = 0;
//                    shooter.shooterShoot = false;
//                    transfer.isDetecting = true;
//                    transfer.ballLeftSmall();
//                    transfer.emptyBalls();
//                    intake.spinnerMacro = false;
//                })
//
//                .build();
//    }

    //only Counterclockwise rotation
    public StateMachine getShootAllMachine (Robot robot){
        Shooter shooter = robot.shooter;
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        return new StateMachineBuilder()
                .state(shootStates.INIT)
                .transition(()->(currentGamepad1.x && !previousGamepad1.x), shootStates.PRESPIN)

                .state(shootStates.PRESPIN)
                .onEnter(()-> {
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    shooter.shooterShoot = true;
                    transfer.isDetecting = false;
                    if(transfer.desiredRotate == 1){
                        transfer.ballLeft();
                    } else if (transfer.desiredRotate == 2){
                        transfer.ballRight();
                    }
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN)
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN)

                .state(shootStates.CLUTCHDOWN)
                .onEnter(()-> {
                    transfer.max = 0.275;
                    transfer.setClutchBarelyDown();
                })
                .transitionTimed(clutchDownTime, shootStates.WAIT1)

                .state(shootStates.WAIT1)
                .transition(()-> shooter.isFarShot(), shootStates.SPIN1)
                .transition(() -> !shooter.isFarShot(), shootStates.SPIN)

                .state(shootStates.SPIN)
                .onEnter(()->{
                    transfer.max = 0.2;
                    transfer.ballLeft();
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget(), shootStates.SPIN3)
                .transitionTimed(spinTime, shootStates.SPIN3)

                .state(shootStates.SPIN1)
                .onEnter(()-> {
                    transfer.ballLeftSmall();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN1)
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN1)

                .state(shootStates.CLUTCHDOWN1)
                .onEnter(()->transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.SPIN2)
                .onExit(()->transfer.setClutchBarelyDown())

                .state(shootStates.SPIN2)
                .onEnter(()-> {
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.CLUTCHDOWN2)
                .transitionTimed(spinUpTimeout, shootStates.CLUTCHDOWN2)

                .state(shootStates.CLUTCHDOWN2)
                .onEnter(()->transfer.setClutchDownFar())
                .transitionTimed(clutchDownFarTime, shootStates.SPIN3)
                .onExit(()-> {
                    transfer.setClutchBarelyDown();
                    transfer.ballRightSmall();
                })

                .state(shootStates.SPIN3)
                .onEnter(()-> {
                    transfer.max = 0.275;
                    transfer.ballLeftSmall();
                    transfer.ballLeft();
                })
                .transition(()-> transfer.spindexAtTarget() && shooter.isAtRPM(), shootStates.WAIT2)
                .transitionTimed(spinUpTimeout, shootStates.WAIT2)
                .onExit(()-> transfer.setClutchDownFar())

                .state(shootStates.WAIT2)
                .transitionTimed(clutchDownFarTime, shootStates.INIT)
                .onExit(()->{
                    transfer.setClutchUp();
                    transfer.ballRightSmall();
                    intake.spinnerMacroTarget = 0;
                    shooter.shooterShoot = false;
                    transfer.isDetecting = true;
                    transfer.emptyBalls();
                    intake.spinnerMacro = false;
                    transfer.max = 0.4;
                })

                .build();
    }

    public StateMachine getClutchSuperMachine (Robot robot){
        Transfer transfer = robot.transfer;
        Intake intake = robot.intake;
        Shooter shooter = robot.shooter;
        return new StateMachineBuilder()
                .state(clutchStates.INIT)
                .transition(()->(currentGamepad1.a && !previousGamepad1.a), clutchStates.SPIN)

                .state(clutchStates.SPIN)
                .onEnter(()-> {
                    transfer.ballRightSmall();
                    shooter.shooterShoot = true;
                    intake.spinnerMacro = true;
                    intake.spinnerMacroTarget = 0.95;
                    transfer.isDetecting = false;
                })
                .transition(()->transfer.spindexAtTarget() && shooter.isAtRPM(), clutchStates.CLUTCHDOWN)
                .transitionTimed(spinUpTimeout, clutchStates.CLUTCHDOWN)

                .state(clutchStates.CLUTCHDOWN)
                .onEnter(()-> {
                    transfer.setClutchDown();
                })
                .transitionTimed(clutchDownTime, clutchStates.CLUTCHUP)
                .onExit(()-> transfer.setClutchDownFar())

                .state(clutchStates.CLUTCHUP)
                .transitionTimed(clutchDownFarTime, clutchStates.INIT)
                .onExit(()->{
                    intake.spinnerMacro = false;
                    intake.spinnerMacroTarget = 0;
                    transfer.setClutchUp();
                    transfer.ballLeftSmall();
                    shooter.shooterShoot = false;
                    transfer.isDetecting = true;
                    transfer.ballList[1] = "E";
                })

                .build();
    }

    public StateMachine getSpindexResetMachine (Robot robot){
        Transfer transfer = robot.transfer;
        return new StateMachineBuilder()
                .state(resetStates.INIT)
                .transition(()->(currentGamepad1.b && !previousGamepad1.b), resetStates.SPIN)

                .state(resetStates.SPIN)
                .onEnter(()->{
                    transfer.useSpindexPID = false;
                    transfer.isDetecting = false;
                    transfer.spindexManualPow = 0.075;
                    transfer.spindexTarget = 0;
                })
                .transition(()-> transfer.isRed, resetStates.RESET)

                .state(resetStates.RESET)
                .onEnter(()->{
                    transfer.spindexManualPow = 0;
                    transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    transfer.useSpindexPID = true;
                })
                .transitionTimed(0.1, resetStates.SPIN2)
                .onExit(()-> transfer.spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER))

                .state(resetStates.SPIN2)
                .onEnter(()->transfer.spindexTarget += fromRed)
                .transition(()->transfer.spindexAtTarget(), resetStates.RESET2)

                .state(resetStates.RESET2)
                .onEnter(()->{
                    transfer.spindexTarget = 0;
                    transfer.spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })
                .transitionTimed(0.1, resetStates.INIT)
                .onExit(()-> {
                    transfer.spindex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    transfer.isDetecting = true;
                })

                .build();
    }
}
