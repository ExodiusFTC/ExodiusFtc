package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.OpModes.tuningconfig.TargetDerivatives;

import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.control.feedforward.SimpleFeedforward;
import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.robot.Mechanism;

/**
 * Turret driven by two CR servos (servo2 mirrors servo1). Angle (degrees) is fed in with
 * setCurrentPos() from the encoder on the intakeMotor port.
 *
 * Closed loop, every periodic():
 *   power = PID(target - angle) + feedforward(targetVel, targetAccel)
 * where targetVel/targetAccel come from TargetDerivatives. Gains are public static so they can
 * be edited live in FTC Dashboard; the PID/feedforward objects are rebuilt when a gain changes.
 */
@Config
public class TurretServos implements Mechanism {

    public static double kP = 0.015;
    public static double kI = 0.0;
    public static double kD = 0.0008;
    public static double kS = 0.001;
    public static double kV = 0.001;
    public static double kA = 0.0;

    public static double maxPower = 1.0;
    // Flip to -1 if positive power moves the turret toward a LOWER encoder angle.
    public static double outputSign = -1.0;
    // Soft limits in degrees (encoder frame). SET THESE TO YOUR REAL RANGE.
    public static double minAngle = -180.0;
    public static double maxAngle = 180.0;

    public static Pose botPose;

    // ---- Auto-aim ----
    // Robot y >= splitY aims at the upper goal point, otherwise at the lower one (field inches).
    public static double splitY = 72.0;
    public static double upperGoalX = 58.0;
    public static double upperGoalY = 85.0;
    public static double lowerGoalX = 58.0;
    public static double lowerGoalY = 59.0;
    // +1 if a positive turret angle turns counter-clockwise (same direction as Pedro heading).
    // -1 if positive turret angle turns clockwise (this robot: clockwise increases the angle).
    public static double angleSign = -1.0;
    // Turret encoder angle (deg) when the turret points straight out the front of the robot.
    // 0 if you zero the encoder with the turret facing forward.
    public static double turretZeroOffset = 0.0;

    public double Pos1;
    public double targPos;
    public double currentPos;

    private boolean closedLoop = false;
    private double error = 0;
    private double output = 0;
    private double aimGoalX = 0;
    private double aimGoalY = 0;
    private double aimFieldAngleDeg = 0;

    private final TargetDerivatives targetDeriv = new TargetDerivatives();
    private PIDController turretPID = new PIDController(kP, kI, kD);
    private SimpleFeedforward turretFeedforward = new SimpleFeedforward(kS, kV, kA);
    private double usedKP = kP, usedKI = kI, usedKD = kD, usedKS = kS, usedKV = kV, usedKA = kA;

    NextCRServo turretServo1 = new NextCRServo("turretServo1");
    NextCRServo turretServo2 = new NextCRServo("turretServo2");

    public TurretServos() {}

    // ---------- manual ----------
    public Command run() {
        return instant(() -> turretServo1.setPower(1)).requiring(turretServo1);
    }
    public Command stop() {
        return instant(() -> turretServo1.setPower(0)).requiring(turretServo1);
    }
    public Command runOpposite() {
        return instant(() -> turretServo1.setPower(-1)).requiring(turretServo1);
    }
    /** Open-loop power; turns closed-loop off. */
    public void setManualPower(double power) {
        closedLoop = false;
        turretServo1.setPower(power);
    }

    // ---------- closed loop ----------
    public void setTargetPos(double target) { targPos = target; }
    public void setCurrentPos(double POS) { currentPos = POS; }
    public void setBotPose(Pose robotposition) { botPose = robotposition; }

    public void setClosedLoop(boolean enabled) {
        if (enabled && !closedLoop) {
            turretPID = new PIDController(kP, kI, kD);   // fresh PID state
            targetDeriv.reset();
        }
        closedLoop = enabled;
    }

    public double getError() { return error; }
    public double getOutput() { return output; }

    // ---------- auto-aim ----------
    private static double normalize180(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    /** Picks the equivalent angle (deg + 360k) that is inside the limits and closest to reference. */
    private static double wrapIntoRange(double desired, double reference) {
        double best = Double.NaN;
        double bestDist = Double.MAX_VALUE;
        for (int k = -3; k <= 3; k++) {
            double c = desired + 360.0 * k;
            if (c < minAngle || c > maxAngle) continue;
            double d = Math.abs(c - reference);
            if (d < bestDist) {
                best = c;
                bestDist = d;
            }
        }
        if (Double.isNaN(best)) {   // no equivalent angle fits: use the nearest limit
            best = Math.max(minAngle, Math.min(maxAngle, normalize180(desired)));
        }
        return best;
    }

    /**
     * Turret angle (deg, encoder frame) that points at the goal, given the robot's field pose.
     * x and y in field inches, heading in radians (Pedro: counter-clockwise from field +x).
     * y >= splitY aims at (upperGoalX, upperGoalY), otherwise at (lowerGoalX, lowerGoalY).
     */
    public double calculateAimAngle(double x, double y, double heading) {
        boolean upper = y >= splitY;
        aimGoalX = upper ? upperGoalX : lowerGoalX;
        aimGoalY = upper ? upperGoalY : lowerGoalY;

        double fieldAngle = Math.atan2(aimGoalY - y, aimGoalX - x);
        aimFieldAngleDeg = Math.toDegrees(fieldAngle);

        // angle of the goal relative to the front of the robot (CCW positive)
        double relative = normalize180(Math.toDegrees(fieldAngle - heading));

        double desired = angleSign * relative + turretZeroOffset;
        return wrapIntoRange(desired, currentPos);
    }

    /** Same, using the pose last given to setBotPose(). Returns the current target if there is none. */
    public double calculateAimAngle() {
        if (botPose == null) return targPos;
        return calculateAimAngle(botPose.x(), botPose.y(), botPose.heading());
    }

    /** Call every loop: points the turret at the goal using the closed-loop controller. */
    public void aimAtGoal() {
        setTargetPos(calculateAimAngle());
        setClosedLoop(true);
    }

    public double getAimGoalX() { return aimGoalX; }
    public double getAimGoalY() { return aimGoalY; }
    public double getAimFieldAngle() { return aimFieldAngleDeg; }

    @Override
    public void periodic() {
        Pos1 = currentPos;
        targetDeriv.update(targPos);

        if (closedLoop) {
            if (kP != usedKP || kI != usedKI || kD != usedKD) {
                turretPID = new PIDController(kP, kI, kD);
                usedKP = kP; usedKI = kI; usedKD = kD;
            }
            if (kS != usedKS || kV != usedKV || kA != usedKA) {
                turretFeedforward = new SimpleFeedforward(kS, kV, kA);
                usedKS = kS; usedKV = kV; usedKA = kA;
            }

            double target = Math.max(minAngle, Math.min(maxAngle, targPos));
            error = target - currentPos;

            double u = turretPID.calculate(error)
                    + turretFeedforward.calculate(targetDeriv.getVelocity(), targetDeriv.getAcceleration());

            if (currentPos >= maxAngle && u > 0) u = 0;   // never push past a limit
            if (currentPos <= minAngle && u < 0) u = 0;

            u = Math.max(-Math.abs(maxPower), Math.min(Math.abs(maxPower), u));
            output = u;
            turretServo1.setPower(outputSign * u);
        } else {
            error = 0;
            output = 0;
        }

        turretServo2.setPower(turretServo1.getPower());
    }
}