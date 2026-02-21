package org.firstinspires.ftc.teamcode.config.autoUtil;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.subsystems.Robot;
import org.firstinspires.ftc.teamcode.config.utility.GlobalVariables;

public class AutoMotifTracker {
    public static final int TAG_MOTIF_1 = 21;
    public static final int TAG_MOTIF_2 = 22;
    public static final int TAG_MOTIF_3 = 23;

    private final Robot robot;
    private final Alliance alliance;
    private final Range range;
    private final ElapsedTime timer = new ElapsedTime();
    private final double timeoutSeconds;

    private int acquiredMotifId = -1;

    public AutoMotifTracker(Robot robot, Alliance alliance, Range range, double timeoutSeconds) {
        this.robot = robot;
        this.alliance = alliance;
        this.range = range;
        this.timeoutSeconds = timeoutSeconds;
    }

    public void reset() {
        timer.reset();
    }

    public int getAcquiredMotifId() {
        return acquiredMotifId;
    }

    public boolean hasTimedOut() {
        return timer.seconds() >= timeoutSeconds;
    }

    public boolean hasMotifOrTimedOut() {
        if (seesAnyMotifTag()) {
            return true;
        }
        return hasTimedOut();
    }

    public boolean hasVisibleMotif() {
        return seesAnyMotifTag();
    }

    public void resolveMotif(boolean preloadComplete) {
        if (robot == null || robot.outtake == null || robot.outtake.vision == null) {
            return;
        }

        if (range == Range.CLOSE_RANGE && !preloadComplete) {
            int seenId = robot.outtake.vision.getCurrentTagId();
            if (alliance == Alliance.BLUE) {
                acquiredMotifId = (seenId == TAG_MOTIF_1) ? TAG_MOTIF_3 : (seenId - 1);
            } else {
                acquiredMotifId = (seenId == TAG_MOTIF_3) ? TAG_MOTIF_1 : (seenId + 1);
            }
        } else {
            acquiredMotifId = robot.outtake.vision.getCurrentTagId();
        }

        boolean validMotif = isMotifId(acquiredMotifId);
        if (validMotif) {
            if (acquiredMotifId == TAG_MOTIF_1) {
                GlobalVariables.setMotif(GlobalVariables.MotifPattern.GPP);
            } else if (acquiredMotifId == TAG_MOTIF_2) {
                GlobalVariables.setMotif(GlobalVariables.MotifPattern.PGP);
            } else if (acquiredMotifId == TAG_MOTIF_3) {
                GlobalVariables.setMotif(GlobalVariables.MotifPattern.PPG);
            }
        }
    }

    private boolean seesAnyMotifTag() {
        return robot != null
                && robot.outtake != null
                && robot.outtake.vision != null
                && (robot.outtake.vision.seesTag(TAG_MOTIF_1)
                    || robot.outtake.vision.seesTag(TAG_MOTIF_2)
                    || robot.outtake.vision.seesTag(TAG_MOTIF_3));
    }

    private boolean isMotifId(int id) {
        return id == TAG_MOTIF_1 || id == TAG_MOTIF_2 || id == TAG_MOTIF_3;
    }
}
