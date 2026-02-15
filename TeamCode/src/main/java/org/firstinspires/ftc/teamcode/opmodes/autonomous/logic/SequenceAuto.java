package org.firstinspires.ftc.teamcode.opmodes.autonomous.logic;

import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;

/**
 * Auto variation that runs an explicit absolute-row sequence.
 * Sequence format follows R: P, row1, row2, ...
 */
public abstract class SequenceAuto extends BaseAuto {

    protected SequenceAuto(Alliance alliance) {
        super(alliance);
    }

    @Override
    protected AutoSpec getSpec() {
        return buildSpec();
    }

    protected abstract AutoSpec buildSpec();

    @Override
    protected StateMachine buildAutoMachine() {
        return buildStandardStateMachine();
    }
}
