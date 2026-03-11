package org.firstinspires.ftc.teamcode.opmodes.autonomous.logic;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;

/**
 * Path-only autonomous variant runner using a provided AutoSpec.
 */
public class SequenceAutoPathTesting extends BaseAutoPathTesting {
    private final AutoSpec spec;

    public SequenceAutoPathTesting(Alliance alliance, AutoSpec spec) {
        super(alliance);
        this.spec = spec;
    }

    @Override
    protected AutoSpec getSpec() {
        return spec;
    }

    @Override
    protected boolean shouldUseCurvedRow2GoToPickup() {
        return spec.range == Range.CLOSE_RANGE
                && spec.closeLoopEnabled
                && spec.rowSequence.length == 2
                && spec.rowSequence[0] == 2
                && spec.rowSequence[1] == 1;
    }
}
