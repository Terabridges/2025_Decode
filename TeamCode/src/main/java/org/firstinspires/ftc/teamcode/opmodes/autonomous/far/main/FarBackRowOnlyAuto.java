package org.firstinspires.ftc.teamcode.opmodes.autonomous.far.main;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.AutoSpec;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.SequenceAuto;

public class FarBackRowOnlyAuto extends SequenceAuto {
    private static final AutoSpec SPEC = AutoSpec.withBackRowLoopCycles(
            Range.LONG_RANGE, false, true, 2, 4);

    public FarBackRowOnlyAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec buildSpec() {
        return SPEC;
    }
}
