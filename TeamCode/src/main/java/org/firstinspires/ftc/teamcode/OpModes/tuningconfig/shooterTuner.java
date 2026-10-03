package org.firstinspires.ftc.teamcode.OpModes.tuningconfig;

import static dev.nextftc.units.Units.RotationsPerMinute;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.Trigger;

@Config
@NextTeleop(name = "Shooter Tuner")
public class shooterTuner extends NextOpMode {
    public static double targetVel = 0;
    public static double kP = 0;
    public static double kS = 0;
    public static double kV = 0;

    private final Robot robot;
    private int loops = 0;

    public shooterTuner(Robot robot) {
        super(robot);
        this.robot = robot;
        Scheduler.reset();
        // Tell NextFTC to also send telemetry to FTC Dashboard
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        Trigger.Companion.getDefaultEventLoop().clear();
    }

    @Override
    public void periodic() {
        loops++;

        robot.getShooter().getShooterMotor().getVelocityConstants().setKP(kP);
        robot.getShooter().getShooterMotor().getVelocityConstants().setKV(kV);
        robot.getShooter().getShooterMotor().getVelocityConstants().setKS(kS);
        robot.getShooter().setTargetVelocity(targetVel);

        Telemetry.log("loops", loops);
        Telemetry.log("target vel", targetVel);
        Telemetry.log("current vel", robot.getShooter().getShooterMotor().getEncoderVelocity().into(RotationsPerMinute));
        Telemetry.log("kP", kP);
        Telemetry.log("kS", kS);
        Telemetry.log("kV", kV);
        // No update() needed: NextFTC calls it automatically after periodic()
    }
}