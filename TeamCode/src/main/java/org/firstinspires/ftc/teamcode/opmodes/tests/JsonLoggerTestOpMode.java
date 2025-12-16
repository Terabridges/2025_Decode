package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.content.Context;
import android.net.wifi.WifiManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.utility.JsonLogger;

import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitLinearOpMode;
import org.psilynx.psikit.ftc.PsiKitOpMode;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "JsonLogger Test")
public class JsonLoggerTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Context ctx = hardwareMap.appContext;
        // Print the likely URL for the logs UI to telemetry to help locate the server
        try {
            WifiManager wm = (WifiManager) ctx.getSystemService(Context.WIFI_SERVICE);
            int ip = wm.getConnectionInfo().getIpAddress();
            final String ipStr = String.format("%d.%d.%d.%d", (ip & 0xff), (ip >> 8 & 0xff), (ip >> 16 & 0xff), (ip >> 24 & 0xff));
            telemetry.addData("Logs UI", "http://%s:8081/", ipStr);
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Logs UI", "unable to determine IP");
            telemetry.update();
        }
        JsonLogger logger = new JsonLogger(ctx, "JsonLoggerTest");
        logger.setMetadata("mode", "TeleOp");
        logger.createTable("samples");

        waitForStart();

        while (opModeIsActive()) {
            Map<String,Object> r = new HashMap<>();
            r.put("left_stick_y", gamepad1.left_stick_y);
            r.put("right_stick_y", gamepad1.right_stick_y);
            r.put("timestamp", System.currentTimeMillis());
            logger.addTableRow("samples", r);
            telemetry.addData("Count", "logged");
            telemetry.update();
            sleep(50);
            if (!opModeIsActive()) break;
        }

        logger.addEvent("opmodeStopped", null);
        logger.save();

    }
}
