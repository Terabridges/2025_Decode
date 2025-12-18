package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.logging.PsiKitDriverStationLogger;
import org.firstinspires.ftc.teamcode.logging.PsiKitMotorLogger;
import org.firstinspires.ftc.teamcode.logging.PsiKitPinpointV2Logger;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.ftc.PsiKitLinearOpMode;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Base class for TeleOp OpModes that use PsiKit logging.
 *
 * <p>This centralizes the PsiKit setup and the HardwareMap "priming" calls needed so
 * {@code processHardwareInputs()} logs devices under {@code /HardwareMap/...}.
 */
public abstract class PsiKitLoggingLinearOpMode extends PsiKitLinearOpMode {

    protected final PsiKitDriverStationLogger driverStationLogger = new PsiKitDriverStationLogger();
    protected final PsiKitMotorLogger motorLogger = new PsiKitMotorLogger();
    protected final PsiKitPinpointV2Logger pinpointLogger = new PsiKitPinpointV2Logger();

    /** Raw FTC references captured before PsiKit wrapping. */
    protected HardwareMap rawHardwareMap;
    protected Gamepad rawGamepad1;
    protected Gamepad rawGamepad2;

    /** Capture raw FTC references (call before {@link #psiKitSetupAndPrimeHardwareLogging()}). */
    protected final void captureRawFtcReferences() {
        if (rawHardwareMap == null) {
            rawHardwareMap = hardwareMap;
        }
        if (rawGamepad1 == null) {
            rawGamepad1 = gamepad1;
        }
        if (rawGamepad2 == null) {
            rawGamepad2 = gamepad2;
        }
    }

    /**
     * Call once during init, before starting logging.
     */
    protected final void psiKitSetupAndPrimeHardwareLogging() {
        psiKitSetup();
        primePsiKitHardwareLogging();
    }

    /**
     * Standard PsiKit startup sequence for TeleOp logging.
     *
     * <p>Call after {@link #captureRawFtcReferences()}.
     */
    protected final void startPsiKitLogging(int rlogPort) {
        // If the prior OpMode was force-stopped, PsiKit may still be "running".
        try { Logger.end(); } catch (Exception ignored) {}
        Logger.reset();

        psiKitSetupAndPrimeHardwareLogging();

        // Use raw gamepads for control behavior; use PsiKit wrappers only for logging.
        this.gamepad1 = rawGamepad1;
        this.gamepad2 = rawGamepad2;

        Logger.addDataReceiver(new RLOGServer(rlogPort));
        Logger.addDataReceiver(new RLOGWriter(defaultLogFilename()));
        Logger.start();
    }

    protected final void endPsiKitLogging() {
        try { Logger.end(); } catch (Exception ignored) {}
    }

    protected String defaultLogFilename() {
        return this.getClass().getSimpleName()
                + "_log_"
                + new SimpleDateFormat("yyyyMMdd_HHmmss_SSS").format(new Date())
                + ".rlog";
    }

    /**
     * Common per-loop logging that should happen after {@link Logger#periodicBeforeUser()}.
     */
    protected final void logPsiKitInputsOncePerLoop() {
        processHardwareInputs();
        driverStationLogger.log(gamepad1, gamepad2);
    }

    /**
     * Optional extra logging from raw devices that should not be wrapped.
     */
    protected final void logRawHardwareOncePerLoop() {
        if (rawHardwareMap == null) {
            return;
        }
        motorLogger.logAll(rawHardwareMap);
        pinpointLogger.logAll(rawHardwareMap);
    }

    private void primePsiKitHardwareLogging() {
        // We intentionally call getAll() on the *wrapped* HardwareMap (this.hardwareMap).
        // This causes PsiKit's HardwareMapWrapper to create and register wrappers for devices,
        // which are then logged by processHardwareInputs().
        try { hardwareMap.getAll(DcMotor.class); } catch (Throwable ignored) {}
        try { hardwareMap.getAll(Servo.class); } catch (Throwable ignored) {}
        try { hardwareMap.getAll(CRServo.class); } catch (Throwable ignored) {}
        try { hardwareMap.getAll(DigitalChannel.class); } catch (Throwable ignored) {}
        try { hardwareMap.getAll(AnalogInput.class); } catch (Throwable ignored) {}
        try { hardwareMap.getAll(VoltageSensor.class); } catch (Throwable ignored) {}
    }
}
