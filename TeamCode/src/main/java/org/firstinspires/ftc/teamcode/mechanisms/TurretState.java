package org.firstinspires.ftc.teamcode.mechanisms;

/** Carries the turret angle from one OpMode to the next (static survives until the app restarts). */
public final class TurretState {
    public static double savedAngle = 0;
    public static boolean hasSaved = false;

    private TurretState() {}

    public static void save(double angle) {
        savedAngle = angle;
        hasSaved = true;
    }

    public static void clear() {
        hasSaved = false;
    }
}