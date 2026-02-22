package org.firstinspires.ftc.teamcode.opmodes.autonomous.far.other;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.AutoSpec;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.SequenceAuto;

public class FarBackrowReleaseAuto extends SequenceAuto {
    private static final AutoSpec SPEC = new AutoSpec(Range.LONG_RANGE, true, false, true, 4, 3, 2);

    public FarBackrowReleaseAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec buildSpec() {
        return SPEC;
    }
}
