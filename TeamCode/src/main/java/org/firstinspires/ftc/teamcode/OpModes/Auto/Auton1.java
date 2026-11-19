package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.OpModes.Auto.paths.Paths1;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@Config
@NextAutonomous(name = "Red Auto 1")
public class Auton1 extends NextOpMode {
    private final Robot robot;
    private final Paths1 paths = new Paths1();
    public static Pose startingPose = new Pose(56, 8, Math.toRadians(180));


    public static double targetVel = 520;
    public static double targetVel2 = 680;
    public static Pose botPose;


    public Auton1(Robot robot) {
        super(robot);
        this.robot = robot;
        telemetry.setMsTransmissionInterval(10);
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
        robot.getIntake().resetEncoder();
        robot.getPedro().follower().setPose(startingPose);
        Scheduler.reset();
    }

    private Command autoRoutine(Follower follower) {
        return sequential(
                waitMs(3000),
                robot.getIntake().run(),
                waitMs(1000),
                follow(follower, paths.firstCycle()),
                follow(follower, paths.firstShootForSecondTip())

                // Add mechanism commands here.
        );
    }
    @Override
    public void disabledPeriodic(){
        robot.getPedro().follower().setPose(startingPose);

        robot.intakeUp().schedule();
        // Zero the turret encoder wherever the turret sits during init (like BioTele does),
        // so place the turret facing straight forward before pressing start.
        robot.getIntake().resetEncoder();

        // Use the follower pose directly: the static botPose is still null until periodic() runs.
        Pose pose = robot.getPedro().follower().pose();
        robot.getTurretServos().setCurrentPos(robot.getIntake().getActualAngle());
        Telemetry.log("Actual angle", robot.getIntake().getActualAngle());
        Telemetry.log("Target Angle", robot.getTurretServos().calculateAimAngle(pose.x(), pose.y(), pose.heading()));
    }
    @Override
    public void start() {
        robot.getShooter().setTargetVelocity(targetVel);
        robot.getShooter().setTargetVelocity2(targetVel2);

        Follower follower = robot.getPedro().follower();

        follower.setPose(paths.startPose());

        autoRoutine(follower).schedule();


    }

    @Override
    public void periodic() {
        botPose = robot.getPedro().follower().pose();

        robot.getShooter().setTargetVelocity(targetVel);
        robot.getShooter().setTargetVelocity2(targetVel2);

        // Turret auto-aim: feed in the current angle first so the aim calc uses this loop's reading.
        double actualAngle = robot.getIntake().getActualAngle();
        robot.getTurretServos().setCurrentPos(actualAngle);
        double aimAngle = robot.getTurretServos().calculateAimAngle(botPose.x(), botPose.y(), botPose.heading());
        robot.getTurretServos().setTargetPos(aimAngle);
        robot.getTurretServos().setClosedLoop(true);

        Telemetry.log("Actual angle", actualAngle);
        Telemetry.log("Target Angle", aimAngle);
        Telemetry.log("Turret error", robot.getTurretServos().getError());
        Telemetry.log("Bot Pose", botPose.toString());
    }

    @Override
    public void end() {
        robot.getTurretServos().setManualPower(0);   // closed loop off, turret stops
    }
}