package org.firstinspires.ftc.teamcode.OpModes.tuningconfig;

/** Estimates the velocity (units/s) and acceleration (units/s^2) of a changing target. */
public class TargetDerivatives {
    private final double velAlpha;   // 1.0 = no smoothing, smaller = smoother but more delay
    private final double accelAlpha;

    private long lastNanos = 0;
    private double lastTarget = 0;
    private double velocity = 0;
    private double acceleration = 0;

    public TargetDerivatives() {
        this(0.3, 0.2);
    }

    public TargetDerivatives(double velAlpha, double accelAlpha) {
        this.velAlpha = velAlpha;
        this.accelAlpha = accelAlpha;
    }

    /** Call once per loop with the current target. */
    public void update(double target) {
        long now = System.nanoTime();
        if (lastNanos == 0) {                 // first call: nothing to compare against
            lastNanos = now;
            lastTarget = target;
            return;
        }
        double dt = (now - lastNanos) / 1e9;
        if (dt < 1e-3) return;                // ignore calls that are too close together

        double rawVel = (target - lastTarget) / dt;
        double newVel = velocity + velAlpha * (rawVel - velocity);

        double rawAccel = (newVel - velocity) / dt;
        acceleration += accelAlpha * (rawAccel - acceleration);
        velocity = newVel;

        lastTarget = target;
        lastNanos = now;
    }

    public double getVelocity()     { return velocity; }
    public double getAcceleration() { return acceleration; }

    /** Call when you stop tracking (or after a large target jump) so old data doesn't linger. */
    public void reset() {
        lastNanos = 0;
        velocity = 0;
        acceleration = 0;
    }
}
