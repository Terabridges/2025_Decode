package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.MainAuto.changes;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.MainAuto.drawCurrent;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.MainAuto.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.MainAuto.follower;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.MainAuto.telemetryM;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.utility.Drawing;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp(name = "MainAuto", group = "Auto")
public class MainAuto extends SelectableOpMode
{

    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public MainAuto() {
        super("Select Auto", s -> {
            s.folder("Blue Side", b -> {
                b.add("Move Only B", MoveB::new);
                b.add("Preload Only B", PreloadB::new);
                b.add("1 Row B", OneRowB::new);
                b.add("2 Row B", TwoRowB::new);
                b.add("3 Row B", ThreeRowB::new);
            });
            s.folder("Red Side", r -> {
                r.add("Move Only R", MoveR::new);
                r.add("Preload Only R", PreloadR::new);
                r.add("1 Row R", OneRowR::new);
                r.add("2 Row R", TwoRowR::new);
                r.add("3 Row R", ThreeRowR::new);
            });
        });
    }

    @Override
    public void onSelect()
    {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void onLog(List<String> lines) {}

    public static void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(poseHistory);
        drawCurrent();
    }

}

class BaseAuto extends OpMode{

    public Pose startPose = new Pose(0, 0, 0.0); //Configure

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {
        telemetryM.debug("Auto: " + this.getClass().getSimpleName());
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    @Override
    public void start() {
        follower.setStartingPose(startPose);
        Tuning.follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();

        // Auto ends when we’re done with the last path.
        if (!follower.isBusy())
        {
            requestOpModeStop();
        }
    }
}

//BLUE SIDE AUTOS

class MoveB extends BaseAuto {

}

class PreloadB extends BaseAuto {

}

class OneRowB extends BaseAuto {

}

class TwoRowB extends BaseAuto {

}

class ThreeRowB extends BaseAuto {

}

//RED SIDE AUTOS

class MoveR extends BaseAuto {

}

class PreloadR extends BaseAuto {

}

class OneRowR extends BaseAuto {

}

class TwoRowR extends BaseAuto {

}

class ThreeRowR extends BaseAuto {

}