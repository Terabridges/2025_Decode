package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.drawCurrent;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.follower;
import static org.firstinspires.ftc.teamcode.opmodes.autonomous.SelectableAuto.telemetryM;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Alliance;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Mode;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.Range;
import org.firstinspires.ftc.teamcode.config.autoUtil.Enums.AutoStates;
import org.firstinspires.ftc.teamcode.config.autoUtil.AutoPoses;

class MainAuto extends OpMode {

    //Path Gen
    public Pose startPose;
    AutoPoses ap = new AutoPoses();
    public PathChain GoToPickup, Pickup, GoToScore, GoToLoad;

    //Enums
    private Alliance alliance;
    private Range range;
    private Mode mode;

    //Other Variables
    int rowIndex = 0;

    MainAuto(Alliance alliance, Range range, Mode mode) {
        this.alliance = alliance;
        this.range = range;
        this.mode = mode;
        startPose = ap.findStartPose(alliance, range);
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {
        telemetryM.debug("Auto: " + this.getClass().getSimpleName());
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    @Override
    public void start() {
        follower.setStartingPose(startPose);
        follower.update();
    }

    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();

        // Auto ends when we’re done with the last path.
        if (!follower.isBusy())
        {
            requestOpModeStop();
        }
    }

    public void buildPaths(String type) {
        follower.update();
        Pose currentPose = follower.getPose();

        if (type.equals("goToPickup")) {
            GoToPickup = buildLinearPath(currentPose, ap.getPickupStart(alliance, range, rowIndex));
        }
        else if (type.equals("pickup")) {
            Pickup = buildLinearPath(currentPose, ap.getPickupEnd(alliance, range, rowIndex));
        }
        else if (type.equals("goToScore")) {
            GoToScore = buildLinearPath(currentPose, ap.getScore(alliance, range));
        }
        else if (type.equals("goToLoad")) {
            GoToLoad = buildLinearPath(currentPose, ap.getLoad(alliance));
        }
        else { //build all
            GoToPickup = buildLinearPath(currentPose, ap.getPickupStart(alliance, range, rowIndex));
            Pickup = buildLinearPath(currentPose, ap.getPickupEnd(alliance, range, rowIndex));
            GoToScore = buildLinearPath(currentPose, ap.getScore(alliance, range));
            GoToLoad = buildLinearPath(currentPose, ap.getLoad(alliance));
            //Note: Could make a mode that builds with previous path end as start
        }
    }

    public PathChain buildLinearPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public PathChain buildCurvedPath(Pose start, Pose control, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }
}