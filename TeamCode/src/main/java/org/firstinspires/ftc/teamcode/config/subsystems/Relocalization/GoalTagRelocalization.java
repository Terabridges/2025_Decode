package org.firstinspires.ftc.teamcode.config.subsystems.Relocalization;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.config.subsystems.Outtake.Turret;
import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

public class GoalTagRelocalization implements Subsystem {
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID = 24;
    private static final double METERS_TO_INCHES = 39.3701;

    public static double relocalizeTurretToleranceDeg = 2.5;
    public static int relocalizeMinTagCount = 1;
    public static double relocalizeMaxStdDevMeters = 1.25;
    public static long relocalizeMaxResultStalenessMs = 250;
    public static double relocalizeExecuteTimeoutSec = 3.0;
    public static double relocalizeFlashDurationSec = 0.9;
    public static double relocalizeFlashPeriodSec = 0.16;

    private enum GoalTagRelocalizePhase {
        IDLE,
        PRIMED,
        EXECUTING,
        FLASH_SUCCESS,
        FLASH_FAIL
    }

    public static class GoalTagRelocalizeResult {
        public final boolean success;
        public final int tagId;
        public final String reason;
        public final Pose followerPoseBefore;
        public final Pose relocalizedPose;

        public GoalTagRelocalizeResult(boolean success, int tagId, String reason, Pose followerPoseBefore, Pose relocalizedPose) {
            this.success = success;
            this.tagId = tagId;
            this.reason = reason;
            this.followerPoseBefore = followerPoseBefore;
            this.relocalizedPose = relocalizedPose;
        }
    }

    private static class RelocalizeAttempt {
        public final GoalTagRelocalizeResult result;
        public final boolean retryable;

        public RelocalizeAttempt(GoalTagRelocalizeResult result, boolean retryable) {
            this.result = result;
            this.retryable = retryable;
        }
    }

    private final Intake intake;
    private final Outtake outtake;

    private boolean goalTagRelocalizePrimed = false;
    private double primedForwardTurretDeg = Turret.turretForwardDeg;
    private GoalTagRelocalizePhase goalTagRelocalizePhase = GoalTagRelocalizePhase.IDLE;
    private final ElapsedTime goalTagRelocalizeTimer = new ElapsedTime();
    private GoalTagRelocalizeResult latestGoalTagRelocalizeResult;

    public GoalTagRelocalization(Intake intake, Outtake outtake) {
        this.intake = intake;
        this.outtake = outtake;
    }

    public GoalTagRelocalizeResult primeGoalTagRelocalization() {
        return primeGoalTagRelocalization(Turret.turretForwardDeg);
    }

    public GoalTagRelocalizeResult primeGoalTagRelocalization(double forwardTurretDeg) {
        primedForwardTurretDeg = outtake.turret.normalizeDegrees(forwardTurretDeg);
        goalTagRelocalizePrimed = true;
        goalTagRelocalizePhase = GoalTagRelocalizePhase.PRIMED;
        goalTagRelocalizeTimer.reset();
        outtake.turret.setTurretDegree(forwardTurretDeg);
        Pose before = snapshotPose(follower != null ? follower.getPose() : null);
        latestGoalTagRelocalizeResult = new GoalTagRelocalizeResult(true, -1, "Relocalization primed: turret moving to forward reference", before, null);
        return latestGoalTagRelocalizeResult;
    }

    public GoalTagRelocalizeResult executeGoalTagRelocalization() {
        return executeGoalTagRelocalization(primedForwardTurretDeg);
    }

    public GoalTagRelocalizeResult executeGoalTagRelocalization(double forwardTurretDeg) {
        Pose before = snapshotPose(follower != null ? follower.getPose() : null);
        if (!goalTagRelocalizePrimed) {
            latestGoalTagRelocalizeResult = new GoalTagRelocalizeResult(false, -1, "Not primed. Press the prime button first.", before, null);
            return latestGoalTagRelocalizeResult;
        }
        return startGoalTagRelocalizationExecution(forwardTurretDeg);
    }

    public GoalTagRelocalizeResult stepGoalTagRelocalization() {
        return stepGoalTagRelocalization(primedForwardTurretDeg);
    }

    public GoalTagRelocalizeResult stepGoalTagRelocalization(double forwardTurretDeg) {
        if (!goalTagRelocalizePrimed) {
            return primeGoalTagRelocalization(forwardTurretDeg);
        }
        return startGoalTagRelocalizationExecution(forwardTurretDeg);
    }

    public boolean isGoalTagRelocalizationPrimed() {
        return goalTagRelocalizePrimed;
    }

    public void cancelGoalTagRelocalizationPrime() {
        goalTagRelocalizePrimed = false;
        goalTagRelocalizePhase = GoalTagRelocalizePhase.IDLE;
        restoreSpindexLights();
    }

    public GoalTagRelocalizeResult getLatestGoalTagRelocalizeResult() {
        return latestGoalTagRelocalizeResult;
    }

    public String getGoalTagRelocalizePhaseName() {
        return goalTagRelocalizePhase.name();
    }

    public void clearGoalTagRelocalizeResult() {
        latestGoalTagRelocalizeResult = null;
    }

    public double getPrimedForwardTurretDeg() {
        return primedForwardTurretDeg;
    }

    public boolean isControllingLights() {
        return goalTagRelocalizePhase != GoalTagRelocalizePhase.IDLE;
    }

    @Override
    public void update() {
        updateGoalTagRelocalizationStateAndLights();
    }

    @Override
    public void toInit() {
        goalTagRelocalizePrimed = false;
        goalTagRelocalizePhase = GoalTagRelocalizePhase.IDLE;
        latestGoalTagRelocalizeResult = null;
    }

    private GoalTagRelocalizeResult startGoalTagRelocalizationExecution(double forwardTurretDeg) {
        Pose before = snapshotPose(follower != null ? follower.getPose() : null);
        if (!goalTagRelocalizePrimed) {
            latestGoalTagRelocalizeResult = new GoalTagRelocalizeResult(false, -1, "Not primed. Press the prime button first.", before, null);
            return latestGoalTagRelocalizeResult;
        }
        if (goalTagRelocalizePhase == GoalTagRelocalizePhase.EXECUTING) {
            latestGoalTagRelocalizeResult = new GoalTagRelocalizeResult(false, -1, "Relocalization already in progress", before, null);
            return latestGoalTagRelocalizeResult;
        }
        primedForwardTurretDeg = outtake.turret.normalizeDegrees(forwardTurretDeg);
        goalTagRelocalizePhase = GoalTagRelocalizePhase.EXECUTING;
        goalTagRelocalizeTimer.reset();
        latestGoalTagRelocalizeResult = new GoalTagRelocalizeResult(false, -1, "Relocalization attempt started", before, null);
        return latestGoalTagRelocalizeResult;
    }

    private RelocalizeAttempt attemptGoalTagRelocalizationOnce(double forwardTurretDeg) {
        Pose before = snapshotPose(follower != null ? follower.getPose() : null);
        if (follower == null) {
            return new RelocalizeAttempt(new GoalTagRelocalizeResult(false, -1, "Follower not initialized", before, null), false);
        }
        if (follower.getPose() == null) {
            return new RelocalizeAttempt(new GoalTagRelocalizeResult(false, -1, "Follower pose unavailable", before, null), false);
        }

        outtake.turret.setTurretDegree(forwardTurretDeg);
        if (!isTurretNearForward(forwardTurretDeg)) {
            return new RelocalizeAttempt(new GoalTagRelocalizeResult(false, -1, "Turret not yet at forward reference", before, null), true);
        }

        outtake.vision.updateRobotOrientationDeg(Math.toDegrees(follower.getPose().getHeading()));

        int tagId = outtake.vision.getVisibleGoalTagId();
        if (tagId != BLUE_GOAL_TAG_ID && tagId != RED_GOAL_TAG_ID) {
            return new RelocalizeAttempt(new GoalTagRelocalizeResult(false, -1, "No goal tag (20/24) visible", before, null), true);
        }

        LLResult latest = outtake.vision.latest;
        String qualityIssue = getRelocalizationQualityIssue(latest);
        if (qualityIssue != null) {
            return new RelocalizeAttempt(new GoalTagRelocalizeResult(false, tagId, qualityIssue, before, null), true);
        }

        Pose3D llPose = outtake.vision.getLatestMt2Pose();
        if (llPose == null) {
            return new RelocalizeAttempt(new GoalTagRelocalizeResult(false, tagId, "Goal tag visible but no Limelight MT2 pose", before, null), true);
        }

        double xIn = llPose.getPosition().x * METERS_TO_INCHES;
        double yIn = llPose.getPosition().y * METERS_TO_INCHES;
        double headingRad = Math.toRadians(llPose.getOrientation().getYaw(AngleUnit.DEGREES));
        Pose relocalized = new Pose(xIn, yIn, headingRad);
        follower.setPose(relocalized);
        return new RelocalizeAttempt(
                new GoalTagRelocalizeResult(true, tagId, "Relocalized from goal tag MT2 pose", before, snapshotPose(relocalized)),
                false
        );
    }

    private Pose snapshotPose(Pose pose) {
        if (pose == null) {
            return null;
        }
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private String getRelocalizationQualityIssue(LLResult latest) {
        if (latest == null) {
            return "No Limelight frame available";
        }
        if (!latest.isValid()) {
            return "Latest Limelight frame is invalid";
        }
        if (latest.getStaleness() > relocalizeMaxResultStalenessMs) {
            return "Limelight frame is stale";
        }
        if (latest.getBotposeTagCount() < relocalizeMinTagCount) {
            return "Insufficient AprilTag count for MT2";
        }
        double[] stddevMt2 = latest.getStddevMt2();
        if (stddevMt2 != null && stddevMt2.length >= 2) {
            if (Double.isFinite(stddevMt2[0]) && stddevMt2[0] > relocalizeMaxStdDevMeters) {
                return "MT2 X stddev too high";
            }
            if (Double.isFinite(stddevMt2[1]) && stddevMt2[1] > relocalizeMaxStdDevMeters) {
                return "MT2 Y stddev too high";
            }
        }
        return null;
    }

    private boolean isTurretNearForward(double forwardTurretDeg) {
        double measured = outtake.turret.getMappedEncoderTurretDegrees();
        double currentDeg = Double.isFinite(measured) ? measured : outtake.turret.getCurrentDegrees();
        double errorDeg = wrapSignedDegrees(forwardTurretDeg - currentDeg);
        return Math.abs(errorDeg) <= Math.abs(relocalizeTurretToleranceDeg);
    }

    private double wrapSignedDegrees(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private void updateGoalTagRelocalizationStateAndLights() {
        switch (goalTagRelocalizePhase) {
            case IDLE:
                return;
            case PRIMED:
                outtake.turret.setTurretDegree(primedForwardTurretDeg);
                setAllLights(isAnyGoalTagVisible() ? "yellow" : "red");
                return;
            case EXECUTING:
                RelocalizeAttempt attempt = attemptGoalTagRelocalizationOnce(primedForwardTurretDeg);
                latestGoalTagRelocalizeResult = attempt.result;
                if (attempt.result.success) {
                    goalTagRelocalizePrimed = false;
                    goalTagRelocalizePhase = GoalTagRelocalizePhase.FLASH_SUCCESS;
                    goalTagRelocalizeTimer.reset();
                    return;
                }
                if (!attempt.retryable) {
                    goalTagRelocalizePrimed = false;
                    goalTagRelocalizePhase = GoalTagRelocalizePhase.FLASH_FAIL;
                    goalTagRelocalizeTimer.reset();
                    return;
                }
                if (goalTagRelocalizeTimer.seconds() >= Math.max(0.1, relocalizeExecuteTimeoutSec)) {
                    Pose before = snapshotPose(follower != null ? follower.getPose() : null);
                    latestGoalTagRelocalizeResult = new GoalTagRelocalizeResult(false, -1, "Relocalization timed out", before, null);
                    goalTagRelocalizePrimed = false;
                    goalTagRelocalizePhase = GoalTagRelocalizePhase.FLASH_FAIL;
                    goalTagRelocalizeTimer.reset();
                    return;
                }
                setAllLights(isAnyGoalTagVisible() ? "yellow" : "red");
                return;
            case FLASH_SUCCESS:
                updateFlashLights("green");
                return;
            case FLASH_FAIL:
                updateFlashLights("red");
        }
    }

    private void updateFlashLights(String color) {
        if (goalTagRelocalizeTimer.seconds() >= Math.max(0.1, relocalizeFlashDurationSec)) {
            goalTagRelocalizePhase = GoalTagRelocalizePhase.IDLE;
            restoreSpindexLights();
            return;
        }
        double periodSec = Math.max(0.05, relocalizeFlashPeriodSec);
        long bucket = (long) (goalTagRelocalizeTimer.seconds() / periodSec);
        boolean flashOn = (bucket % 2L) == 0L;
        setAllLights(flashOn ? color : "clear");
    }

    private boolean isAnyGoalTagVisible() {
        int visibleGoalTag = outtake.vision.getVisibleGoalTagId();
        return visibleGoalTag == BLUE_GOAL_TAG_ID || visibleGoalTag == RED_GOAL_TAG_ID;
    }

    private void setAllLights(String color) {
        intake.lights.setFrontLight(color);
        intake.lights.setMiddleLight(color);
        intake.lights.setBackLight(color);
    }

    private void restoreSpindexLights() {
        if (intake == null || intake.spindex == null || intake.spindex.ballList == null || intake.spindex.ballList.length < 3) {
            return;
        }
        applyBallSlotLight(intake.spindex.ballList[0], 0);
        applyBallSlotLight(intake.spindex.ballList[1], 1);
        applyBallSlotLight(intake.spindex.ballList[2], 2);
    }

    private void applyBallSlotLight(String slotValue, int index) {
        String value = (slotValue == null) ? "E" : slotValue;
        if ("E".equals(value)) {
            setSlotLight(index, "clear");
            return;
        }
        if ("B".equals(value)) {
            if (index == 0) intake.lights.setFrontLightAlliance();
            else if (index == 1) intake.lights.setMiddleLightAlliance();
            else intake.lights.setBackLightAlliance();
            return;
        }
        if ("P".equals(value)) {
            setSlotLight(index, "purple");
            return;
        }
        if ("G".equals(value)) {
            setSlotLight(index, "green");
            return;
        }
        setSlotLight(index, "clear");
    }

    private void setSlotLight(int index, String color) {
        if (index == 0) {
            intake.lights.setFrontLight(color);
        } else if (index == 1) {
            intake.lights.setMiddleLight(color);
        } else {
            intake.lights.setBackLight(color);
        }
    }
}
