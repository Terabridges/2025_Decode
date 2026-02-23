package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BenchA Loop Baseline", group = "Benchmark")
public class BenchA_LoopBaseline extends QuickLoopBenchBase {
    @Override
    protected String benchName() {
        return "BenchA_Baseline_NoPsiKit_NoRobot";
    }
}
