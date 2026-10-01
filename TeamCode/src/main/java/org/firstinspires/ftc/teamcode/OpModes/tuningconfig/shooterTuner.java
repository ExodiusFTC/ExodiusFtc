package org.firstinspires.ftc.teamcode.OpModes.tuningconfig;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;

import static dev.nextftc.units.Units.Rotations;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;

@Config
@NextTeleop(name = "Shooter Tuner")
public class shooterTuner extends NextOpMode {
    public static double targetVel = 0;
    public static double kP = 0;
    public static double kS = 0;
    public static double kV = 0;
    private final Robot robot;
    public shooterTuner(Robot robot){
        super(robot);
        this.robot = robot;
        Trigger.Companion.getDefaultEventLoop().clear();

    }
    @Override
    public void start() {

        CommandGamepad gp1 = new CommandGamepad(gamepad1);

//        gp1.dpadRight().onTrue(instant(() -> kS+=0.0001));
//        gp1.dpadLeft().onTrue(instant(() -> kS-=0.0001));
//        gp1.dpadUp().onTrue(instant(() -> kV += 0.01));
//        gp1.dpadDown().onTrue(instant(() -> kV -= 0.01));
//        gp1.leftBumper().onTrue(instant(() -> kP += 0.01));
//        gp1.rightBumper().onTrue(instant(() -> kP -= 0.01));
//        gp1.a().onTrue(instant(() -> targetVel += 50));
//        gp1.b().onTrue(instant(() -> targetVel -= 50));

    }
    @Override
    public void periodic(){

        robot.getShooter().getShooterMotor().getVelocityConstants().setKP(kP);
        robot.getShooter().getShooterMotor().getVelocityConstants().setKV(kV);
        robot.getShooter().getShooterMotor().getVelocityConstants().setKS(kS);
        robot.getShooter().setTargetVelocity(targetVel);


        telemetry.addData("target vel", targetVel);
        telemetry.addData("current vel", robot.getShooter().getShooterMotor().getEncoderVelocity().into(RotationsPerMinute));
        telemetry.addData("kP", kP);
        telemetry.addData("kS", kS);
        telemetry.addData("kV", kV);
        telemetry.update();
    }


}
