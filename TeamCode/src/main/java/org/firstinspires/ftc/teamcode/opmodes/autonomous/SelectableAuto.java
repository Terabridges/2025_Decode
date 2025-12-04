package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Mode;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.ShotPlan;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
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
                    m.folder("All Selected Range", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED));
                        s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("4 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.FOUR_ROW, ShotPlan.ALL_SELECTED));
                        s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED));
                        s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED));
                    });
                    m.folder("Closest Point", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("4 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.FOUR_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT));
                        s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT));
                    });
                });
                p.folder("Long Range", m -> {
                    m.folder("All Selected Range", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED));
                        s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("4 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.FOUR_ROW, ShotPlan.ALL_SELECTED));
                        s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED));
                        s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED));
                    });
                    m.folder("Closest Point", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("4 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.FOUR_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT));
                        s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT));
                    });
                });
            });
            c.folder("Red Side", p -> {
                p.folder("Close Range", m -> {
                    m.folder("All Selected Range", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED));
                        s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("4 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.FOUR_ROW, ShotPlan.ALL_SELECTED));
                        s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED));
                        s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED));
                    });
                    m.folder("Closest Point", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("4 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.FOUR_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT));
                        s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT));
                    });
                });
                p.folder("Long Range", m -> {
                    m.folder("All Selected Range", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED));
                        s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("4 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.FOUR_ROW, ShotPlan.ALL_SELECTED));
                        s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED));
                        s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED));
                    });
                    m.folder("Closest Point", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("4 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.FOUR_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT));
                        s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT));
                    });
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
