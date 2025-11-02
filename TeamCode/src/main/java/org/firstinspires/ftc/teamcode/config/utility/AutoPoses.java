package org.firstinspires.ftc.teamcode.config.utility;


import com.pedropathing.geometry.Pose;

public class AutoPoses {

    int pickupMode = 0;
    public void setPickupMode(int i) {
        if (i < 3 && i >= 0) {
            pickupMode = i;
        }
    }

    //------------------ POSES ------------------

    //Starting Poses
    public final Pose closeStartPose = new Pose();
    public final Pose farStartPose = new Pose();

    //Scoring Poses
    public final Pose longRangeScore = new Pose();
    public final Pose closeRangeScore = new Pose();

    //Pickup Poses [Head on, offset below, offset above]
    public final Pose[] pickupR1Start = {new Pose(), new Pose(), new Pose()};
    public final Pose[] pickupR2Start = {new Pose(), new Pose(), new Pose()};
    public final Pose[] pickupR3Start = {new Pose(), new Pose(), new Pose()};
    public final Pose[] pickupR4Start = {new Pose(), new Pose(), new Pose()};

    public final Pose[] pickupR1End = {new Pose(), new Pose(), new Pose()};
    public final Pose[] pickupR2End = {new Pose(), new Pose(), new Pose()};
    public final Pose[] pickupR3End = {new Pose(), new Pose(), new Pose()};
    public final Pose[] pickupR4End = {new Pose(), new Pose(), new Pose()};

    public final Pose[] pickupStart = {pickupR1Start[pickupMode], pickupR2Start[pickupMode], pickupR3Start[pickupMode], pickupR4Start[pickupMode]};
    public final Pose[] pickupEnd = {pickupR1End[pickupMode], pickupR2End[pickupMode], pickupR3End[pickupMode], pickupR4End[pickupMode]};

    //Loading Pose
    public final Pose loadingPose = new Pose();

}
