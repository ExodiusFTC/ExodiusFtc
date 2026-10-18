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
    public static double kP = 0.001;
    public static double kS = 0.09;
    public static double kV = 0.0000445;

    public static double targetVel2 = 0;
    public static double kP2 = 0.0001;
    public static double kS2 = 0.05;
    public static double kV2 = 0.000024;
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

        robot.getShooter().getShooterMotor().getVelocityConstants().setKP(kP);
        robot.getShooter().getShooterMotor().getVelocityConstants().setKV(kV);
        robot.getShooter().getShooterMotor().getVelocityConstants().setKS(kS);
        robot.getShooter().setTargetVelocity(targetVel);

        robot.getShooter().getCounterMotor().getVelocityConstants().setKP(kP2);
        robot.getShooter().getCounterMotor().getVelocityConstants().setKV(kV2);
        robot.getShooter().getCounterMotor().getVelocityConstants().setKS(kS2);
        robot.getShooter().setTargetVelocity2(targetVel2);


        Telemetry.log("target vel", targetVel);
        Telemetry.log("current vel", robot.getShooter().getShooterMotor().getEncoderVelocity().into(RotationsPerMinute));
        Telemetry.log("kP", kP);
        Telemetry.log("kS", kS);
        Telemetry.log("kV", kV);
        Telemetry.log("------------");
        Telemetry.log("target vel 2", targetVel2);
        Telemetry.log("current vel 2", robot.getShooter().getCounterMotor().getEncoderVelocity().into(RotationsPerMinute));
        Telemetry.log("kP 2", kP2);
        Telemetry.log("kS 2", kS2);
        Telemetry.log("kV 2", kV2);
        // No update() needed: NextFTC calls it automatically after periodic()
    }
}