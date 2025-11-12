package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.drawCurrent;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.follower;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.telemetryM;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.autoUtil.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Mode;
import org.firstinspires.ftc.teamcode.config.autoUtil.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;
import org.firstinspires.ftc.teamcode.config.utility.Drawing;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name = "MainAuto", group = "Auto")
public class SelectableAuto extends SelectableOpMode
{
    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public SelectableAuto() {
        super("Select Auto", c -> {
            c.folder("Blue Side", p -> {
                p.folder("Close Range", m -> {
                    m.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.ONE_ROW));
                    m.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.TWO_ROW));
                    m.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.THREE_ROW));
                    m.add("4 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.FOUR_ROW));
                    m.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.MOVE_ONLY));
                    m.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY));
                });
                p.folder("Long Range", m -> {
                    m.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.ONE_ROW));
                    m.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.TWO_ROW));
                    m.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.THREE_ROW));
                    m.add("4 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.FOUR_ROW));
                    m.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.MOVE_ONLY));
                    m.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.PRELOAD_ONLY));
                });
            });
            c.folder("Red Side", p -> {
                p.folder("Close Range", m -> {
                    m.add("1 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.ONE_ROW));
                    m.add("2 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.TWO_ROW));
                    m.add("3 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.THREE_ROW));
                    m.add("4 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.FOUR_ROW));
                    m.add("Move Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.MOVE_ONLY));
                    m.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY));
                });
                p.folder("Long Range", m -> {
                    m.add("1 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.ONE_ROW));
                    m.add("2 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.TWO_ROW));
                    m.add("3 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.THREE_ROW));
                    m.add("4 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.FOUR_ROW));
                    m.add("Move Only", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.MOVE_ONLY));
                    m.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.PRELOAD_ONLY));
                });
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

class MainAuto extends OpMode {

    //Path Gen
    public Pose startPose;
    AutoPoses ap = new AutoPoses();
    public PathChain GoToPickup, Pickup, GoToScore, GoToLoad;

    //Enums
    private Alliance alliance;
    private Range range;
    private Mode mode;

    MainAuto(Alliance alliance, Range range, Mode mode) {
        this.alliance = alliance;
        this.range = range;
        this.mode = mode;
        startPose = ap.findStartPose(alliance, range);
    }

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
        follower.update();
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

    public void buildPaths() {
        follower.update();
        Pose currentPose = follower.getPose();

        GoToPickup = buildLinearPath(currentPose, ap.getPickupStart(alliance, range, mode));
        Pickup = buildLinearPath(currentPose, ap.getPickupEnd(alliance, range, mode));
        GoToScore = buildLinearPath(currentPose, ap.getScore(alliance, range));
        GoToLoad = buildLinearPath(currentPose, ap.getLoad(alliance));

    }

    public PathChain buildLinearPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public PathChain buildCurvedPath(Pose start, Pose control, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}