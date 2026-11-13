package org.firstinspires.ftc.teamcode.OpModes.tuningconfig;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.Trigger;

/**
 * Set targetVel (shooter wheel) and targetVel2 (counter wheel) in FTC Dashboard.
 * Gains (kS/kV/kP and kS2/kV2/kP2), switchFraction, reengageFraction, bangPower and
 * counterSign are edited on the Shooter config, so what you tune here is what the robot uses.
 */
@Config
@NextTeleop(name = "Shooter Tuner")
public class shooterTuner extends NextOpMode {
    public static double targetVel = 0;
    public static double targetVel2 = 0;
    private final Robot robot;

    public shooterTuner(Robot robot) {
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
        Shooter shooter = robot.getShooter();
        shooter.setTargetVelocity(targetVel);
        shooter.setTargetVelocity2(targetVel2);

        Telemetry.log("target vel", targetVel);
        Telemetry.log("measured vel", shooter.getMeasuredVelocity());
        Telemetry.log("error", targetVel - shooter.getMeasuredVelocity());
        Telemetry.log("mode", shooter.getMode().toString());
        Telemetry.log("shooter power", shooter.getPower());

        Telemetry.log("target vel 2", targetVel2);
        Telemetry.log("counter measured vel", shooter.getMeasuredVelocity2());
        Telemetry.log("error 2", targetVel2 - shooter.getMeasuredVelocity2());
        Telemetry.log("mode 2", shooter.getMode2().toString());
        Telemetry.log("counter power", shooter.getPower2());
        // No update() needed: NextFTC calls it automatically after periodic()
    }
}