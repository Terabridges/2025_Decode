package org.firstinspires.ftc.teamcode.psikit;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.robolectric.annotation.Implementation;
import org.robolectric.annotation.Implements;

@Implements(AppUtil.class)
public class ShadowAppUtil {

    @Implementation
    public void loadLibrary(String name) {
        // no-op (prevents Robolectric from trying to load native libs)
    }
}
