package org.firstinspires.ftc.teamcode.OpModes.Teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
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
    public static Pose botPose;
    public static Pose startingPose = new Pose(0, 0, Math.toRadians(0));
    public static double targetVel = 0;

    public static double targetVel2 = 0;
    private final Robot robot;
    public BioTele(Robot robot){
        super(robot);
        this.robot = robot;
        telemetry.setMsTransmissionInterval(10);
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
        gp2.rightBumper().onTrue(robot.getTurretServos().run());
        gp2.rightBumper().onFalse(robot.getTurretServos().stop());
        gp2.leftBumper().onTrue(robot.getTurretServos().runOpposite());
        gp2.leftBumper().onFalse(robot.getTurretServos().stop());








    }



//    @Override
//    public void disabledPeriodic(){
//
//    }
    @Override
    public void periodic(){
        botPose = robot.getPedro().follower().pose();
        robot.getTurretServos().setBotPose(botPose);
        if(gamepad2.xWasPressed()){
            robot.getIntake().resetEncoder();
        }
        double turretAngle = robot.getIntake().getCurrentAngle();
        double turretVel = robot.getIntake().getCurrentVel();
        Telemetry.log("Status", "Running");
        Telemetry.log("Status", "Running");
        robot.getShooter().setTargetVelocity(targetVel);
        robot.getShooter().setTargetVelocity2(targetVel2);
        Telemetry.log("current vel", robot.getShooter().getShooterMotor().getEncoderVelocity());
        Telemetry.log("current vel 2", robot.getShooter().getCounterMotor().getEncoderVelocity());
        Telemetry.log("turretAngle", turretAngle);
        Telemetry.log("turretVel", turretVel);
        Telemetry.log("Actual angle", robot.getIntake().getActualAngle());
        Telemetry.log("Bot Pose", botPose.toString());

//        Telemetry.log("current angle", robot.getTurret().getCurrentAngle());
//        Telemetry.log("current vel", robot.getTurret().getCurrentVel());
//        Telemetry.log("Encoder Ticks", robot.getEncoder().getCurrentTicks());
//        Telemetry.log("Encoder Velocity", robot.getEncoder().getTickVelocity());
    }

    @Override
    public void end(){

    }
}
