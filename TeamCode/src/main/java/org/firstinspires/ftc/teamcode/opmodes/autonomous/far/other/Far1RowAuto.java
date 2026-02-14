package org.firstinspires.ftc.teamcode.opmodes.autonomous.far.other;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.AutoSpec;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.SequenceAuto;

public class Far1RowAuto extends SequenceAuto {
    private static final AutoSpec SPEC = new AutoSpec(Range.LONG_RANGE, false, false, true, 4);

    public Far1RowAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec buildSpec() {
        return SPEC;
    }
}
