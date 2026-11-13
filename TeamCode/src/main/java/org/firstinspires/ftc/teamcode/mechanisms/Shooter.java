package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

/**
 * Two independent bang-bang + feedforward + P flywheel controllers, one per motor.
 *
 * Each wheel (shooter, counter) runs the same strategy with its own target, measured
 * velocity and kS/kV/kP:
 *  - Far below target: full power (bang-bang spin-up).
 *  - Close to target:  kS + kV * target + kP * error.
 *  - If a shot drops the speed far enough below target, back to full power until it
 *    recovers, then back to feedforward.
 *
 * The counter motor is mounted reversed, so its power and velocity are multiplied by
 * counterSign (-1) to work in a "positive = spinning the right way" frame.
 *
 * Velocity unit is rad/s (same as the old setTargetVelocity). Type targets in the same
 * unit the "measured vel" telemetry shows and tune kV in that unit.
 */
@Config
public class Shooter implements Mechanism {
    public enum Mode { IDLE, BANG, FEEDFORWARD }

    NextMotor shooterMotor = new NextMotor("SH");
    NextMotor counterMotor = new NextMotor("SH2");

    // ---- Shooter wheel gains (live-editable in FTC Dashboard) ----
    public static double kS = 0.1;
    public static double kV = 0.00024;   // PLACEHOLDER: retune. Power per (rad/s).
    public static double kP = 0.01;    // PLACEHOLDER: retune. Power per (rad/s) of error.

    // ---- Counter wheel gains ----
    public static double kS2 = 0.12;
    public static double kV2 = 0.00025;  // PLACEHOLDER: retune.
    public static double kP2 = 0.01;   // PLACEHOLDER: retune.

    // ---- Bang-bang switching (shared by both wheels), fraction of target velocity ----
    // Switch bang -> feedforward once within this fraction of target.
    public static double switchFraction = 0.05;
    // Switch feedforward -> bang if velocity falls this far below target (after a shot).
    // Must be >= switchFraction; the gap is hysteresis that stops chattering.
    public static double reengageFraction = 0.08;
    public static double bangPower = 1.0;

    // -1 because the counter motor is reversed. Applied to both the power sent to it
    // and its measured velocity. If "counter measured vel" reads negative while the
    // wheel spins the correct way, flip this to +1 (and check the motor direction).
    public static double counterSign = -1.0;

    /** Controller state for one flywheel. */
    private static class Wheel {
        double target = 0;
        double measured = 0;
        double power = 0;
        Mode mode = Mode.IDLE;

        void update(double kS, double kV, double kP) {
            if (target <= 0) {
                mode = Mode.IDLE;
                power = 0;
                return;
            }

            double error = target - measured;
            double enter = switchFraction * target;
            double exit = Math.max(reengageFraction, switchFraction) * target;

            if (mode == Mode.FEEDFORWARD) {
                if (error > exit) mode = Mode.BANG;
            } else {
                mode = (error <= enter) ? Mode.FEEDFORWARD : Mode.BANG;
            }

            if (mode == Mode.BANG) {
                power = bangPower;
            } else {
                // feedforward + proportional trim, clamped (never negative: no braking)
                power = kS + kV * target + kP * error;
                power = Math.max(0.0, Math.min(1.0, power));
            }
        }
    }

    private final Wheel shooterWheel = new Wheel();
    private final Wheel counterWheel = new Wheel();

    public void setTargetVelocity(double targetVel) {
        shooterWheel.target = targetVel;
    }

    public void setTargetVelocity2(double targetVel2) {
        counterWheel.target = targetVel2;
    }

    public NextMotor getShooterMotor() {
        return shooterMotor;
    }

    public NextMotor getCounterMotor() {
        return counterMotor;
    }

    public double getMeasuredVelocity() { return shooterWheel.measured; }
    public double getPower() { return shooterWheel.power; }
    public Mode getMode() { return shooterWheel.mode; }

    public double getMeasuredVelocity2() { return counterWheel.measured; }
    public double getPower2() { return counterWheel.power; }
    public Mode getMode2() { return counterWheel.mode; }

    @Override
    public void periodic() {
        shooterWheel.measured = shooterMotor.getEncoderVelocity().into(Units.RadiansPerSecond);
        counterWheel.measured = counterSign
                * counterMotor.getEncoderVelocity().into(Units.RadiansPerSecond);

        shooterWheel.update(kS, kV, kP);
        counterWheel.update(kS2, kV2, kP2);

        shooterMotor.setThrottle(shooterWheel.power);
        counterMotor.setThrottle(counterSign * counterWheel.power);
    }
}