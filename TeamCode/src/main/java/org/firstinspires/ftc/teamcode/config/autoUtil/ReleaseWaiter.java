package org.firstinspires.ftc.teamcode.config.autoUtil;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ReleaseWaiter {
    private final ElapsedTime timer = new ElapsedTime();
    private final double idleSeconds;
    private double idleStart = -1.0;

    public ReleaseWaiter(double idleSeconds) {
        this.idleSeconds = idleSeconds;
    }

    public void reset() {
        timer.reset();
        idleStart = -1.0;
    }

    public boolean isDone(boolean followerIdle, double stateTimeoutSeconds) {
        if (timer.seconds() >= stateTimeoutSeconds) {
            return true;
        }
        if (followerIdle) {
            if (idleStart < 0.0) {
                idleStart = timer.seconds();
            }
            return (timer.seconds() - idleStart) >= idleSeconds;
        }
        idleStart = -1.0;
        return false;
    }
}
