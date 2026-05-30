package org.firstinspires.ftc.teamcode.opmodes.autonomous.far.main;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.AutoSpec;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.SequenceAuto;

public class FarMainBackRowNoLeaveAuto extends SequenceAuto {
    private static final AutoSpec SPEC = AutoSpec.withBackRowLoopCycles(
            Range.LONG_RANGE, false, true, false, 1, 4, 3);

    public FarMainBackRowNoLeaveAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec buildSpec() {
        return SPEC;
    }
}
