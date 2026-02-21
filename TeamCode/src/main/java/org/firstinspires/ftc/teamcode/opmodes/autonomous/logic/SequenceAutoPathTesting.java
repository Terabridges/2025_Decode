package org.firstinspires.ftc.teamcode.opmodes.autonomous.logic;

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
}
