package org.firstinspires.ftc.teamcode.opmodes.autonomous.logic;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name = "MainAutoPathTesting", group = "Auto")
public class SelectableAutoPathTesting extends SelectableOpMode {
    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    private static final AutoSpec CLOSE_PRELOAD_ONLY =
            new AutoSpec(Range.CLOSE_RANGE, false, false, true);
    private static final AutoSpec CLOSE_1_RELEASE =
            new AutoSpec(Range.CLOSE_RANGE, true, false, true, 1);
    private static final AutoSpec CLOSE_1_NO_RELEASE =
            new AutoSpec(Range.CLOSE_RANGE, false, false, true, 1);
    private static final AutoSpec CLOSE_2_RELEASE =
            new AutoSpec(Range.CLOSE_RANGE, true, false, true, 1, 2);
    private static final AutoSpec CLOSE_2_NO_RELEASE =
            new AutoSpec(Range.CLOSE_RANGE, false, false, true, 1, 2);
    private static final AutoSpec CLOSE_3_RELEASE =
            new AutoSpec(Range.CLOSE_RANGE, true, false, true, 1, 2, 3);
    private static final AutoSpec CLOSE_3_NO_RELEASE =
            new AutoSpec(Range.CLOSE_RANGE, false, false, true, 1, 2, 3);
    private static final AutoSpec CLOSE_4_RELEASE =
            new AutoSpec(Range.CLOSE_RANGE, true, false, true, 1, 2, 3, 4);
    private static final AutoSpec CLOSE_4_NO_RELEASE =
            new AutoSpec(Range.CLOSE_RANGE, false, false, true, 1, 2, 3, 4);

    private static final AutoSpec FAR_PRELOAD_ONLY =
            new AutoSpec(Range.LONG_RANGE, false, false, true);
    private static final AutoSpec FAR_1 =
            new AutoSpec(Range.LONG_RANGE, false, false, true, 4);
    private static final AutoSpec FAR_2 =
            new AutoSpec(Range.LONG_RANGE, false, false, true, 4, 3);
    private static final AutoSpec FAR_3 =
            new AutoSpec(Range.LONG_RANGE, false, false, true, 4, 3, 2);
    private static final AutoSpec FAR_3_THEN_2 =
            new AutoSpec(Range.LONG_RANGE, false, false, true, 3, 2);
    private static final AutoSpec FAR_4 =
            new AutoSpec(Range.LONG_RANGE, false, false, true, 4, 3, 2, 1);
    private static final AutoSpec FAR_BACKROW_RELEASE =
            new AutoSpec(Range.LONG_RANGE, true, false, true, 4, 3, 2);
    private static final AutoSpec FAR_BACKROW_ONLY =
            AutoSpec.withBackRowLoopCycles(Range.LONG_RANGE, false, true, 2, 4);
    private static final AutoSpec FAR_BACKROW_PLUS_ONE =
            AutoSpec.withBackRowLoopCycles(Range.LONG_RANGE, false, true, 2, 4, 3);

    public SelectableAutoPathTesting() {
        super("Select Auto Path Test", c -> {
            c.folder("Blue Alliance", p -> {
                p.folder("Close", m -> {
                    m.add("3 Row Release (R: P, 1, 2, 3)", () -> make(Alliance.BLUE, CLOSE_3_RELEASE));
                    m.add("3 Row No Release (R: P, 1, 2, 3)", () -> make(Alliance.BLUE, CLOSE_3_NO_RELEASE));
                    m.folder("Other", s -> {
                        s.add("Preload Only (R: P)", () -> make(Alliance.BLUE, CLOSE_PRELOAD_ONLY));
                        s.folder("1 Row (R: P, 1)", v -> {
                            v.add("Release", () -> make(Alliance.BLUE, CLOSE_1_RELEASE));
                            v.add("No Release", () -> make(Alliance.BLUE, CLOSE_1_NO_RELEASE));
                        });
                        s.folder("2 Row (R: P, 1, 2)", v -> {
                            v.add("Release", () -> make(Alliance.BLUE, CLOSE_2_RELEASE));
                            v.add("No Release", () -> make(Alliance.BLUE, CLOSE_2_NO_RELEASE));
                        });
                        s.folder("4 Row (R: P, 1, 2, 3, 4)", v -> {
                            v.add("Release", () -> make(Alliance.BLUE, CLOSE_4_RELEASE));
                            v.add("No Release", () -> make(Alliance.BLUE, CLOSE_4_NO_RELEASE));
                        });
                    });
                });
                p.folder("Far", m -> {
                    m.add("Back Row Only (R: P, 4, backrow loop)", () -> make(Alliance.BLUE, FAR_BACKROW_ONLY));
                    m.add("Back Row Plus 1 Row (R: P, 4, 3, backrow loop)", () -> make(Alliance.BLUE, FAR_BACKROW_PLUS_ONE));
                    m.add("3 Row (R: P, 4, 3, 2)", () -> make(Alliance.BLUE, FAR_3));
                    m.add("Rows 3 + 2 (R: P, 3, 2)", () -> make(Alliance.BLUE, FAR_3_THEN_2));
                    m.folder("Other", s -> {
                        s.add("Backrow Release (R: P, 4, 3, 2)", () -> make(Alliance.BLUE, FAR_BACKROW_RELEASE));
                        s.add("Preload Only (R: P)", () -> make(Alliance.BLUE, FAR_PRELOAD_ONLY));
                        s.add("1 Row (R: P, 4)", () -> make(Alliance.BLUE, FAR_1));
                        s.add("2 Row (R: P, 4, 3)", () -> make(Alliance.BLUE, FAR_2));
                        s.add("4 Row (R: P, 4, 3, 2, 1)", () -> make(Alliance.BLUE, FAR_4));
                    });
                });
            });
            c.folder("Red Alliance", p -> {
                p.folder("Close", m -> {
                    m.add("3 Row Release (R: P, 1, 2, 3)", () -> make(Alliance.RED, CLOSE_3_RELEASE));
                    m.add("3 Row No Release (R: P, 1, 2, 3)", () -> make(Alliance.RED, CLOSE_3_NO_RELEASE));
                    m.folder("Other", s -> {
                        s.add("Preload Only (R: P)", () -> make(Alliance.RED, CLOSE_PRELOAD_ONLY));
                        s.folder("1 Row (R: P, 1)", v -> {
                            v.add("Release", () -> make(Alliance.RED, CLOSE_1_RELEASE));
                            v.add("No Release", () -> make(Alliance.RED, CLOSE_1_NO_RELEASE));
                        });
                        s.folder("2 Row (R: P, 1, 2)", v -> {
                            v.add("Release", () -> make(Alliance.RED, CLOSE_2_RELEASE));
                            v.add("No Release", () -> make(Alliance.RED, CLOSE_2_NO_RELEASE));
                        });
                        s.folder("4 Row (R: P, 1, 2, 3, 4)", v -> {
                            v.add("Release", () -> make(Alliance.RED, CLOSE_4_RELEASE));
                            v.add("No Release", () -> make(Alliance.RED, CLOSE_4_NO_RELEASE));
                        });
                    });
                });
                p.folder("Far", m -> {
                    m.add("Back Row Only (R: P, 4, backrow loop)", () -> make(Alliance.RED, FAR_BACKROW_ONLY));
                    m.add("Back Row Plus 1 Row (R: P, 4, 3, backrow loop)", () -> make(Alliance.RED, FAR_BACKROW_PLUS_ONE));
                    m.add("3 Row (R: P, 4, 3, 2)", () -> make(Alliance.RED, FAR_3));
                    m.add("Rows 3 + 2 (R: P, 3, 2)", () -> make(Alliance.RED, FAR_3_THEN_2));
                    m.folder("Other", s -> {
                        s.add("Backrow Release (R: P, 4, 3, 2)", () -> make(Alliance.RED, FAR_BACKROW_RELEASE));
                        s.add("Preload Only (R: P)", () -> make(Alliance.RED, FAR_PRELOAD_ONLY));
                        s.add("1 Row (R: P, 4)", () -> make(Alliance.RED, FAR_1));
                        s.add("2 Row (R: P, 4, 3)", () -> make(Alliance.RED, FAR_2));
                        s.add("4 Row (R: P, 4, 3, 2, 1)", () -> make(Alliance.RED, FAR_4));
                    });
                });
            });
        });
    }

    private static SequenceAutoPathTesting make(Alliance alliance, AutoSpec spec) {
        return new SequenceAutoPathTesting(alliance, spec);
    }

    @Override
    public void onSelect() {
        FollowerManager.initFollower(hardwareMap, new Pose());
        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    @Override
    public void onLog(List<String> lines) {}
}
