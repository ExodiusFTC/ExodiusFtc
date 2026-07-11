package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;

public class FeedbackServoSubsystem implements Subsystem {
    public static final FeedbackServoSubsystem INSTANCE = new FeedbackServoSubsystem();

    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double DEFAULT_CACHE_TOLERANCE = 0.01;
    private static final double MIN_LOOP_SECONDS = 1e-3;

    private final FeedbackCRServoEx servo = new FeedbackCRServoEx(
            DEFAULT_CACHE_TOLERANCE,
            () -> ActiveOpMode.hardwareMap().analogInput.get("servo_feedback"),
            () -> ActiveOpMode.hardwareMap().crservo.get("servo")
    );

    private final ControlSystem controller = ControlSystem.builder()
            .posPid(0.8, 0.0, 0.02)
            .basicFF(0.0, 0.0, 0.0)
            .build();

    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean trackerInitialized;
    private double wrappedPositionRadians;
    private double trackedPositionRadians;
    private double trackedVelocityRadiansPerSecond;

    private FeedbackServoSubsystem() {}

    @Override
    public void initialize() {
        wrappedPositionRadians = servo.getCurrentPosition();
        trackedPositionRadians = wrappedPositionRadians;
        trackedVelocityRadiansPerSecond = 0.0;
        trackerInitialized = true;

        controller.reset();
        controller.setGoal(new KineticState(trackedPositionRadians));

        loopTimer.reset();
        servo.setPower(0.0);
    }

    @Override
    public void periodic() {
        updateTrackedState();

        double output = controller.calculate(
                new KineticState(trackedPositionRadians, trackedVelocityRadiansPerSecond)
        );

        servo.setPower(Range.clip(output, -1.0, 1.0));
    }

    public Command runToPosition(double targetRadians) {
        return new RunToPosition(controller, targetRadians, 0.05).requires(this);
    }

    public Command runToPosition(double targetRadians, double toleranceRadians) {
        return new RunToPosition(controller, targetRadians, toleranceRadians).requires(this);
    }

    public void setTargetPosition(double targetRadians) {
        controller.setGoal(new KineticState(targetRadians));
    }

    public double getTargetPosition() {
        return controller.getGoal().getPosition();
    }

    public double getTrackedPositionRadians() {
        return trackedPositionRadians;
    }

    public double getTrackedVelocityRadiansPerSecond() {
        return trackedVelocityRadiansPerSecond;
    }

    public double getWrappedPositionRadians() {
        return servo.getCurrentPosition();
    }

    public void stop() {
        servo.setPower(0.0);
        controller.setGoal(new KineticState(trackedPositionRadians));
    }

    private void updateTrackedState() {
        double currentWrappedPositionRadians = servo.getCurrentPosition();

        if (!trackerInitialized) {
            wrappedPositionRadians = currentWrappedPositionRadians;
            trackedPositionRadians = currentWrappedPositionRadians;
            trackedVelocityRadiansPerSecond = 0.0;
            trackerInitialized = true;
            loopTimer.reset();
            return;
        }

        double deltaRadians = currentWrappedPositionRadians - wrappedPositionRadians;

        if (deltaRadians > Math.PI) {
            deltaRadians -= TWO_PI;
        } else if (deltaRadians < -Math.PI) {
            deltaRadians += TWO_PI;
        }

        double dtSeconds = Math.max(loopTimer.seconds(), MIN_LOOP_SECONDS);

        trackedPositionRadians += deltaRadians;
        trackedVelocityRadiansPerSecond = deltaRadians / dtSeconds;
        wrappedPositionRadians = currentWrappedPositionRadians;

        loopTimer.reset();
    }
}
