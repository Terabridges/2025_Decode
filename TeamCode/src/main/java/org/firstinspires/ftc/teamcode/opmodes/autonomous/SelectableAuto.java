package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Mode;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.ShotPlan;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;

import java.util.ArrayList;
import java.util.List;

@Configurable
@PsiKitAutoLog
@Autonomous(name = "MainAuto", group = "Auto")
public class SelectableAuto extends SelectableOpMode
{
    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public SelectableAuto() {
        super("Select Auto", c -> {
            c.folder("Blue Side", p -> {
                p.folder("Close Range", m -> {
                    m.folder("Release", rel -> {
                        rel.folder("All Selected Range", s -> {
                            s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED, true));
                            s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED, true));
                            s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED, true));
                            s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED, true));
                            s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED, true));
                        });
                        rel.folder("Closest Point", s -> {
                            s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT, true));
                            s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT, true));
                            s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT, true));
                            s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT, true));
                            s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT, true));
                        });
                    });
                    m.folder("No Release", rel -> {
                        rel.folder("All Selected Range", s -> {
                            s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED, false));
                            s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED, false));
                            s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED, false));
                            s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED, false));
                            s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED, false));
                        });
                        rel.folder("Closest Point", s -> {
                            s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT, false));
                            s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT, false));
                            s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT, false));
                            s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT, false));
                            s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT, false));
                        });
                    });
                });
                p.folder("Long Range", m -> {
                    m.folder("All Selected Range", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED));
                        s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED));
                        s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED));
                    });
                    m.folder("Closest Point", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("2 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("3 Row", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("Move Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT));
                        s.add("Preload Only", () -> new MainAuto(Alliance.BLUE, Range.LONG_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT));
                    });
                });
            });
            c.folder("Red Side", p -> {
                p.folder("Close Range", m -> {
                    m.folder("Release", rel -> {
                        rel.folder("All Selected Range", s -> {
                            s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED, true));
                            s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED, true));
                            s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED, true));
                            s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED, true));
                            s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED, true));
                        });
                        rel.folder("Closest Point", s -> {
                            s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT, true));
                            s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT, true));
                            s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT, true));
                            s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT, true));
                            s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT, true));
                        });
                    });
                    m.folder("No Release", rel -> {
                        rel.folder("All Selected Range", s -> {
                            s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED, false));
                            s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED, false));
                            s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED, false));
                            s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED, false));
                            s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED, false));
                        });
                        rel.folder("Closest Point", s -> {
                            s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT, false));
                            s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT, false));
                            s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT, false));
                            s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.MOVE_ONLY, ShotPlan.CLOSEST_POINT, false));
                            s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.CLOSE_RANGE, Mode.PRELOAD_ONLY, ShotPlan.CLOSEST_POINT, false));
                        });
                    });
                });
                p.folder("Long Range", m -> {
                    m.folder("All Selected Range", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.ONE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.TWO_ROW, ShotPlan.ALL_SELECTED));
                        s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.THREE_ROW, ShotPlan.ALL_SELECTED));
                        s.add("Move Only", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.MOVE_ONLY, ShotPlan.ALL_SELECTED));
                        s.add("Preload Only", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.PRELOAD_ONLY, ShotPlan.ALL_SELECTED));
                    });
                    m.folder("Closest Point", s -> {
                        s.add("1 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.ONE_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("2 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.TWO_ROW, ShotPlan.CLOSEST_POINT));
                        s.add("3 Row", () -> new MainAuto(Alliance.RED, Range.LONG_RANGE, Mode.THREE_ROW, ShotPlan.CLOSEST_POINT));
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
        FollowerManager.initFollower(hardwareMap, new Pose());
        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    @Override
    public void onLog(List<String> lines) {}
}
