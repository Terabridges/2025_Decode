package org.firstinspires.ftc.teamcode.opmodes.autonomous.far.main;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.AutoSpec;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.SequenceAuto;

public class FarBackRowPlus1RowAuto extends SequenceAuto {
    private static final boolean LEAVE_AT_ONE_SECOND = true;
    private static final AutoSpec SPEC = AutoSpec.withBackRowLoopCycles(
            Range.LONG_RANGE, false, true, LEAVE_AT_ONE_SECOND, 1, 3, 4);

    public FarBackRowPlus1RowAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec buildSpec() {
        return SPEC;
    }
}
