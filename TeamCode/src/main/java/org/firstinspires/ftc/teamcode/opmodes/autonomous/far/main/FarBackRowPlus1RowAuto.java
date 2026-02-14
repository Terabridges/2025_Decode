package org.firstinspires.ftc.teamcode.opmodes.autonomous.far.main;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.AutoSpec;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.SequenceAuto;

public class FarBackRowPlus1RowAuto extends SequenceAuto {
    private static final AutoSpec SPEC = new AutoSpec(Range.LONG_RANGE, false, true, true, 4, 3);

    public FarBackRowPlus1RowAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec buildSpec() {
        return SPEC;
    }
}
