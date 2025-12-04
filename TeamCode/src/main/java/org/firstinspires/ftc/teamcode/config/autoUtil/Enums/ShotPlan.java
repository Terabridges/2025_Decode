package org.firstinspires.ftc.teamcode.config.autoUtil.Enums;

/** Determines which score pose to use per shot. */
public enum ShotPlan {
    ALL_SELECTED,   // Always use the initially selected range (close or long)
    CLOSEST_POINT   // Use close for rows near, switch to long for farther rows per spec
}
