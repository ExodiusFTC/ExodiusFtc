package org.firstinspires.ftc.teamcode.OpModes.tuningconfig;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.TurretServos;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.Trigger;

/**
 * Turret tuner. The PID + feedforward live in TurretServos; this just sets the target.
 * Angle comes from the encoder on the intakeMotor port (degrees).
 *
 * Dashboard: edit TurretServos gains (kP/kI/kD/kS/kV/kA, outputSign, limits) and this class's
 * enableControl / targetAngle.
 * Gamepad 1: X zeroes the encoder; right stick X drives the turret manually while disabled.
 */
@Config
@NextTeleop(name = "Turret Tuner")
public class turretTuner extends NextOpMode {
    public static boolean enableControl = false;
    public static double targetAngle = 0;     // deg
    public static double manualScale = 0.8;

    private final Robot robot;

    public turretTuner(Robot robot) {
        super(robot);
        this.robot = robot;
        Scheduler.reset();
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        Trigger.Companion.getDefaultEventLoop().clear();
    }

    @Override
    public void periodic() {
        TurretServos turret = robot.getTurretServos();
        Intake intake = robot.getIntake();

        if (gamepad1.xWasPressed()) intake.resetEncoder();

        double angle = intake.getActualAngle();
        turret.setCurrentPos(angle);

        if (enableControl) {
            turret.setTargetPos(targetAngle);
            turret.setClosedLoop(true);
        } else {
            turret.setManualPower(gamepad2.right_stick_x * manualScale);
        }

        Telemetry.log("target angle", targetAngle);
        Telemetry.log("measured angle", angle);
        Telemetry.log("error", turret.getError());
        Telemetry.log("output", turret.getOutput());
    }
}