package org.firstinspires.ftc.teamcode;

// Not on robot yet

public class ASWaitForLauncher extends AS_Action {
    ASAuto auto;


    public ASWaitForLauncher(ASAuto auto) {
        this.auto = auto;
    }

    public void onStart() {}

    public boolean isDone() {
        return this.auto.launcher.isAtVelocity();
    }
}