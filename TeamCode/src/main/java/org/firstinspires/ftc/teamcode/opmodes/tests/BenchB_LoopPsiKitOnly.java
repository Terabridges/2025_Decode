package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "BenchB Loop PsiKit Only", group = "Benchmark")
public class BenchB_LoopPsiKitOnly extends QuickLoopBenchBase {
    @Override
    protected boolean isPsiKitLoopLoggingEnabled() {
        return true;
    }

    @Override
    protected String benchName() {
        return "BenchB_PsiKitOnly_NoRobot";
    }
}
