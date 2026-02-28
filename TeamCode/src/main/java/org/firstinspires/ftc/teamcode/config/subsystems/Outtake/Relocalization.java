package org.firstinspires.ftc.teamcode.config.subsystems.Outtake;

import static org.firstinspires.ftc.teamcode.config.pedroPathing.FollowerManager.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake.Lights;
import org.firstinspires.ftc.teamcode.config.subsystems.Subsystem;

@Configurable
public class Relocalization implements Subsystem {
    private static final double METERS_TO_INCHES = 39.3701;

    public static double turretForwardDeg = Turret.turretForwardDeg;
    public static double turretForwardToleranceDeg = 4.0;
    public static double maxPoseJumpInches = 24.0;
    public static double maxHeadingJumpDeg = 30.0;
    public static double maxStalenessMs = 150.0;
    public static double successFlashMs = 500.0;
    public static double failFlashMs = 500.0;

    public enum State {
        IDLE,
        PREPARED,
        FEEDBACK
    }

    private final Turret turret;
    private final Vision vision;
    private Lights lights;

    private State state = State.IDLE;
    private boolean hadAimLockSnapshot = false;
    private boolean aimLockWasEnabled = false;
    private long feedbackEndMs = 0;
    private String feedbackColor = "clear";
    private boolean lastSuccess = false;
    private String lastReason = "Idle";

    public Relocalization(Turret turret, Vision vision) {
        this.turret = turret;
        this.vision = vision;
    }

    public void setLights(Lights lights) {
        this.lights = lights;
    }

    public void onRelocalizeButtonPress() {
        if (state == State.IDLE) {
            beginPreparedState();
            return;
        }

        if (state == State.PREPARED) {
            attemptRelocalization();
            return;
        }

        if (state == State.FEEDBACK) {
            clearFeedback();
            beginPreparedState();
        }
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public State getState() {
        return state;
    }

    public boolean wasLastSuccess() {
        return lastSuccess;
    }

    public String getLastReason() {
        return lastReason;
    }

    public boolean isPreparedTagVisible() {
        return state == State.PREPARED && isAnyGoalTagVisible();
    }

    private void beginPreparedState() {
        if (!hadAimLockSnapshot) {
            aimLockWasEnabled = turret.isAimLockEnabled();
            hadAimLockSnapshot = true;
        }
        turret.setAimLockEnabled(false);
        state = State.PREPARED;
        lastReason = "Prepared: press B again to relocalize";
    }

    private void attemptRelocalization() {
        if (follower == null || follower.getPose() == null) {
            finishWithFeedback(false, "Follower pose unavailable");
            return;
        }

        Pose followerPose = follower.getPose();
        vision.updateRobotYawDegrees(Math.toDegrees(followerPose.getHeading()));

        int visibleGoalTag = vision.getVisibleGoalTagId();
        if (visibleGoalTag < 0) {
            finishWithFeedback(false, "No goal tag in frame");
            return;
        }

        double turretErrorDeg = Math.abs(wrapSignedDeg(turretForwardDeg - turret.getCurrentDegrees()));
        if (turretErrorDeg > turretForwardToleranceDeg) {
            finishWithFeedback(false, "Turret not settled at forward");
            return;
        }

        Pose3D mt2 = vision.getLatestBotPoseMT2();
        if (mt2 == null) {
            finishWithFeedback(false, "No MT2 pose");
            return;
        }

        double stalenessMs = (vision.latest != null) ? vision.latest.getStaleness() : Double.POSITIVE_INFINITY;
        if (stalenessMs > maxStalenessMs) {
            finishWithFeedback(false, "MT2 stale frame");
            return;
        }

        double xIn = mt2.getPosition().x * METERS_TO_INCHES;
        double yIn = mt2.getPosition().y * METERS_TO_INCHES;
        double headingRad = Math.toRadians(mt2.getOrientation().getYaw(AngleUnit.DEGREES));

        double jumpIn = Math.hypot(xIn - followerPose.getX(), yIn - followerPose.getY());
        double headingJumpDeg = Math.abs(wrapSignedDeg(Math.toDegrees(headingRad - followerPose.getHeading())));
        if (jumpIn > maxPoseJumpInches || headingJumpDeg > maxHeadingJumpDeg) {
            finishWithFeedback(false, "Rejected pose jump");
            return;
        }

        follower.setPose(new Pose(xIn, yIn, headingRad));
        finishWithFeedback(true, "Relocalized from MT2");
    }

    private void finishWithFeedback(boolean success, String reason) {
        lastSuccess = success;
        lastReason = reason;
        // Release turret/aim hold immediately after the attempt result is known.
        restoreAimLock();
        state = State.FEEDBACK;
        feedbackColor = success ? "green" : "red";
        feedbackEndMs = System.currentTimeMillis() + (long) (success ? successFlashMs : failFlashMs);
    }

    private void clearFeedback() {
        feedbackColor = "clear";
        state = State.IDLE;
        restoreAimLock();
        setAllLights("clear");
    }

    private void restoreAimLock() {
        if (!hadAimLockSnapshot) {
            return;
        }
        turret.setAimLockEnabled(aimLockWasEnabled);
        hadAimLockSnapshot = false;
    }

    private boolean isAnyGoalTagVisible() {
        return vision.seesTag(Vision.BLUE_GOAL_TAG_ID) || vision.seesTag(Vision.RED_GOAL_TAG_ID);
    }

    private void setAllLights(String color) {
        if (lights == null) return;
        lights.setFrontLight(color);
        lights.setMiddleLight(color);
        lights.setBackLight(color);
    }

    private static double wrapSignedDeg(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    @Override
    public void toInit() {
        state = State.IDLE;
        feedbackColor = "clear";
        hadAimLockSnapshot = false;
        lastReason = "Idle";
    }

    @Override
    public void update() {
        if (state == State.IDLE) {
            return;
        }

        if (state == State.PREPARED) {
            turret.setAimLockEnabled(false);
            turret.setTurretDegree(turretForwardDeg);
            if (isAnyGoalTagVisible()) {
                setAllLights("yellow");
            } else {
                setAllLights("clear");
            }
            return;
        }

        if (state == State.FEEDBACK) {
            setAllLights(feedbackColor);
            if (System.currentTimeMillis() >= feedbackEndMs) {
                clearFeedback();
            }
        }
    }
}
