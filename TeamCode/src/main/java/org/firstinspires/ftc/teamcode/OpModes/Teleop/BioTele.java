package org.firstinspires.ftc.teamcode.OpModes.Teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;
import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;


@Config
@NextTeleop(name = "BioTele")
public class BioTele extends NextOpMode {
    public static double targetVel = 0;

    public static double targetVel2 = 0;
    private final Robot robot;
    public BioTele(Robot robot){
        super(robot);
        this.robot = robot;
        //this.robot.initHardware(hardwareMap);
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
    }
    @Override
    public void start() {
        Trigger.Companion.getDefaultEventLoop().clear();

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);
        robot.startDrive(gamepad1);
        gp1.rightBumper().onTrue(robot.getIntake().run());
        gp1.rightBumper().onFalse(robot.getIntake().stop());

        gp1.x().onTrue(robot.intakeUp());
        gp1.y().onTrue(robot.intakeDown());




    }



//    @Override
//    public void disabledPeriodic(){
//
//    }
    @Override
    public void periodic(){
        Telemetry.log("Status", "Running");
        Telemetry.log("Status", "Running");
        robot.getShooter().setTargetVelocity(targetVel);
        robot.getShooter().setTargetVelocity2(targetVel2);
        Telemetry.log("current vel", robot.getShooter().getShooterMotor().getEncoderVelocity());
        Telemetry.log("current vel 2", robot.getShooter().getCounterMotor().getEncoderVelocity());

//        Telemetry.log("Encoder Ticks", robot.getEncoder().getCurrentTicks());
//        Telemetry.log("Encoder Velocity", robot.getEncoder().getTickVelocity());
    }
    @Override
    public void end(){

    }
}
