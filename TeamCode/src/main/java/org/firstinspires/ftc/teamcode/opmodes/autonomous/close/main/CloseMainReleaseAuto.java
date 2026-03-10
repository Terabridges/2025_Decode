package org.firstinspires.ftc.teamcode.opmodes.autonomous.close.main;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.AutoSpec;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.logic.SequenceAuto;

@Configurable
public class CloseMainReleaseAuto extends SequenceAuto {
    public static int closeLoopCycles = 1;

    public CloseMainReleaseAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec buildSpec() {
        return AutoSpec.withCloseLoopCycles(
                Range.CLOSE_RANGE,
                true,
                true,
                closeLoopCycles,
                2, 1
        );
    }
}
