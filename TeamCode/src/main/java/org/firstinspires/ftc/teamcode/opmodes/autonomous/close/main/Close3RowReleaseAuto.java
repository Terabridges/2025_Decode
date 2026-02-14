package org.firstinspires.ftc.teamcode.opmodes.autonomous.close.main;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.AutoSpec;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.SequenceAuto;

public class Close3RowReleaseAuto extends SequenceAuto {
    private static final AutoSpec SPEC = new AutoSpec(Range.CLOSE_RANGE, true, false, true, 1, 2, 3);

    public Close3RowReleaseAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec buildSpec() {
        return SPEC;
    }
}
