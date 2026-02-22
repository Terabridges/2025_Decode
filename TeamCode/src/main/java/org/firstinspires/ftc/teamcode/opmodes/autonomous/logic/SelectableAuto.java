package org.firstinspires.ftc.teamcode.opmodes.autonomous.logic;


import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.other.Close1RowNoReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.other.Close1RowReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.other.Close2RowNoReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.other.Close2RowReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.main.Close3RowNoReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.main.Close3RowReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.other.Close4RowNoReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.other.Close4RowReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.close.other.ClosePreloadOnlyNoReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.other.Far1RowAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.other.Far2RowAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.other.Far3RowAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.other.Far4RowAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.main.FarBackRowOnlyAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.main.FarBackRowPlus1RowAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.main.Far3Then2Auto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.other.FarBackrowReleaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.far.other.FarPreloadOnlyAuto;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager;

import java.util.ArrayList;
import java.util.List;

@Configurable
//@PsiKitAutoLog(rlogPort = 5802)
@Autonomous(name = "MainAuto", group = "Auto")
public class SelectableAuto extends SelectableOpMode {
    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public SelectableAuto() {
        super("Select Auto", c -> {
            c.folder("Blue Alliance", p -> {
                p.folder("Close", m -> {
                    m.add("3 Row Release (R: P, 1, 2, 3)", () -> new Close3RowReleaseAuto(Alliance.BLUE));
                    m.add("3 Row No Release (R: P, 1, 2, 3)", () -> new Close3RowNoReleaseAuto(Alliance.BLUE));
                    m.folder("Other", s -> {
                        s.add("Preload Only (R: P)", () -> new ClosePreloadOnlyNoReleaseAuto(Alliance.BLUE));
                        s.folder("1 Row (R: P, 1)", v -> {
                            v.add("Release", () -> new Close1RowReleaseAuto(Alliance.BLUE));
                            v.add("No Release", () -> new Close1RowNoReleaseAuto(Alliance.BLUE));
                        });
                        s.folder("2 Row (R: P, 1, 2)", v -> {
                            v.add("Release", () -> new Close2RowReleaseAuto(Alliance.BLUE));
                            v.add("No Release", () -> new Close2RowNoReleaseAuto(Alliance.BLUE));
                        });
                        s.folder("4 Row (R: P, 1, 2, 3, 4)", v -> {
                            v.add("Release", () -> new Close4RowReleaseAuto(Alliance.BLUE));
                            v.add("No Release", () -> new Close4RowNoReleaseAuto(Alliance.BLUE));
                        });
                    });
                });
                p.folder("Far", m -> {
                    m.add("Back Row Only (R: P, 4, backrow loop)", () -> new FarBackRowOnlyAuto(Alliance.BLUE));
                    m.add("Back Row Plus 1 Row (R: P, 4, 3, backrow loop)", () -> new FarBackRowPlus1RowAuto(Alliance.BLUE));
                    m.add("3 Row (R: P, 4, 3, 2)", () -> new Far3RowAuto(Alliance.BLUE));
                    m.add("Rows 3 + 2 (R: P, 3, 2)", () -> new Far3Then2Auto(Alliance.BLUE));
                    m.folder("Other", s -> {
                        s.add("Backrow Release (R: P, 4, 3, 2)", () -> new FarBackrowReleaseAuto(Alliance.BLUE));
                        s.add("Preload Only (R: P)", () -> new FarPreloadOnlyAuto(Alliance.BLUE));
                        s.add("1 Row (R: P, 4)", () -> new Far1RowAuto(Alliance.BLUE));
                        s.add("2 Row (R: P, 4, 3)", () -> new Far2RowAuto(Alliance.BLUE));
                        s.add("4 Row (R: P, 4, 3, 2, 1)", () -> new Far4RowAuto(Alliance.BLUE));
                    });
                });
            });
            c.folder("Red Alliance", p -> {
                p.folder("Close", m -> {
                    m.add("3 Row Release (R: P, 1, 2, 3)", () -> new Close3RowReleaseAuto(Alliance.RED));
                    m.add("3 Row No Release (R: P, 1, 2, 3)", () -> new Close3RowNoReleaseAuto(Alliance.RED));
                    m.folder("Other", s -> {
                        s.add("Preload Only (R: P)", () -> new ClosePreloadOnlyNoReleaseAuto(Alliance.RED));
                        s.folder("1 Row (R: P, 1)", v -> {
                            v.add("Release", () -> new Close1RowReleaseAuto(Alliance.RED));
                            v.add("No Release", () -> new Close1RowNoReleaseAuto(Alliance.RED));
                        });
                        s.folder("2 Row (R: P, 1, 2)", v -> {
                            v.add("Release", () -> new Close2RowReleaseAuto(Alliance.RED));
                            v.add("No Release", () -> new Close2RowNoReleaseAuto(Alliance.RED));
                        });
                        s.folder("4 Row (R: P, 1, 2, 3, 4)", v -> {
                            v.add("Release", () -> new Close4RowReleaseAuto(Alliance.RED));
                            v.add("No Release", () -> new Close4RowNoReleaseAuto(Alliance.RED));
                        });
                    });
                });
                p.folder("Far", m -> {
                    m.add("Back Row Only (R: P, 4, backrow loop)", () -> new FarBackRowOnlyAuto(Alliance.RED));
                    m.add("Back Row Plus 1 Row (R: P, 4, 3, backrow loop)", () -> new FarBackRowPlus1RowAuto(Alliance.RED));
                    m.add("3 Row (R: P, 4, 3, 2)", () -> new Far3RowAuto(Alliance.RED));
                    m.add("Rows 3 + 2 (R: P, 3, 2)", () -> new Far3Then2Auto(Alliance.RED));
                    m.folder("Other", s -> {
                        s.add("Backrow Release (R: P, 4, 3, 2)", () -> new FarBackrowReleaseAuto(Alliance.RED));
                        s.add("Preload Only (R: P)", () -> new FarPreloadOnlyAuto(Alliance.RED));
                        s.add("1 Row (R: P, 4)", () -> new Far1RowAuto(Alliance.RED));
                        s.add("2 Row (R: P, 4, 3)", () -> new Far2RowAuto(Alliance.RED));
                        s.add("4 Row (R: P, 4, 3, 2, 1)", () -> new Far4RowAuto(Alliance.RED));
                    });
                });
            });
        });
    }

    @Override
    public void onSelect() {
        FollowerManager.initFollower(hardwareMap, new Pose());
        PanelsConfigurables.INSTANCE.refreshClass(this);
    }

    @Override
    public void onLog(List<String> lines) {}
}
