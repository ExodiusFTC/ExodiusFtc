package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "tmpf")
public class tmpf extends NextFTCOpMode {
    private static final double EXTENDED_TICKS = 1700.0;
    private static final double RETRACTED_TICKS = 0.0;

    // Motion constraints for trapezoidal profile (ticks, ticks/s, ticks/s^2)
    private static final double MAX_VEL = 2200.0;
    private static final double MAX_ACCEL = 3500.0;

    private final MotorEx boxTube = new MotorEx("BT").brakeMode();
    private final ElapsedTime profileTimer = new ElapsedTime();

    // Keep existing PIDF architecture; motion profile only provides moving goal states.
    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .basicFF(0.000004, 0, 0)
            .build();

    private TrapezoidProfile profile;
    private double profileStartPosition;
    private double targetPosition = RETRACTED_TICKS;

    private boolean prevA;
    private boolean prevB;

    public tmpf() {
        addComponents(BulkReadComponent.INSTANCE);
    }

    @Override
    public void onInit() {
        profileStartPosition = boxTube.getCurrentPosition();
        targetPosition = profileStartPosition;
        profile = new TrapezoidProfile(0.0, MAX_VEL, MAX_ACCEL);
        controlSystem.setGoal(new KineticState(targetPosition, 0.0, 0.0));
        profileTimer.reset();
    }

    @Override
    public void onUpdate() {
       // updateTargetFromGamepad();
        if (gamepad1.aWasPressed()){
            startProfileTo(EXTENDED_TICKS);
        }
        if (gamepad1.bWasPressed()) {
            startProfileTo(RETRACTED_TICKS);
        }

        double t = profileTimer.seconds();
        MotionState desired = profile.sample(t);

        double desiredPosition = profileStartPosition + desired.position;
        double desiredVelocity = desired.velocity;
        double desiredAcceleration = desired.acceleration;

        controlSystem.setGoal(new KineticState(desiredPosition, desiredVelocity, desiredAcceleration));
        boxTube.setPower(controlSystem.calculate(boxTube.getState()));

        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current", boxTube.getCurrentPosition());
        telemetry.addData("DesiredPos", desiredPosition);
        telemetry.addData("DesiredVel", desiredVelocity);
        telemetry.addData("DesiredAccel", desiredAcceleration);
        telemetry.addData("ProfileTime", t);
    }

//    private void updateTargetFromGamepad() {
//        boolean aPressed = gamepad1.a;
//        boolean bPressed = gamepad1.b;
//
//        if (aPressed && !prevA) {
//            startProfileTo(EXTENDED_TICKS);
//        }
//        if (bPressed && !prevB) {
//            startProfileTo(RETRACTED_TICKS);
//        }
//
//        prevA = aPressed;
//        prevB = bPressed;
//    }

    private void startProfileTo(double newTarget) {
        double currentPosition = boxTube.getCurrentPosition();
        targetPosition = newTarget;
        profileStartPosition = currentPosition;
        profile = new TrapezoidProfile(targetPosition - currentPosition, MAX_VEL, MAX_ACCEL);
        profileTimer.reset();
    }

    private static class MotionState {
        final double position;
        final double velocity;
        final double acceleration;

        MotionState(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    private static class TrapezoidProfile {
        private final double distance;
        private final double direction;
        private final double accel;
        private final double cruiseVelocity;
        private final double tAccel;
        private final double tCruise;
        private final double tTotal;
        private final double dAccel;

        TrapezoidProfile(double distance, double maxVelocity, double maxAccel) {
            this.distance = Math.abs(distance);
            this.direction = Math.signum(distance == 0 ? 1 : distance);
            this.accel = maxAccel;

            double accelTimeCandidate = maxVelocity / maxAccel;
            double accelDistanceCandidate = 0.5 * maxAccel * accelTimeCandidate * accelTimeCandidate;

            if (2.0 * accelDistanceCandidate >= this.distance) {
                // Triangular profile: never reaches max velocity.
                this.tAccel = Math.sqrt(this.distance / maxAccel);
                this.cruiseVelocity = maxAccel * this.tAccel;
                this.tCruise = 0.0;
                this.dAccel = 0.5 * maxAccel * this.tAccel * this.tAccel;
            } else {
                this.tAccel = accelTimeCandidate;
                this.cruiseVelocity = maxVelocity;
                this.dAccel = accelDistanceCandidate;
                double dCruise = this.distance - (2.0 * this.dAccel);
                this.tCruise = dCruise / maxVelocity;
            }

            this.tTotal = (2.0 * this.tAccel) + this.tCruise;
        }

        MotionState sample(double timeSeconds) {
            double t = Range.clip(timeSeconds, 0.0, tTotal);

            double pos;
            double vel;
            double acc;

            if (t <= tAccel) {
                pos = 0.5 * accel * t * t;
                vel = accel * t;
                acc = accel;
            } else if (t <= (tAccel + tCruise)) {
                double tc = t - tAccel;
                pos = dAccel + (cruiseVelocity * tc);
                vel = cruiseVelocity;
                acc = 0.0;
            } else {
                double td = t - tAccel - tCruise;
                pos = dAccel + (cruiseVelocity * tCruise) +
                        (cruiseVelocity * td) - (0.5 * accel * td * td);
                vel = cruiseVelocity - (accel * td);
                acc = -accel;
            }

            return new MotionState(pos * direction, vel * direction, acc * direction);
        }
    }
}
