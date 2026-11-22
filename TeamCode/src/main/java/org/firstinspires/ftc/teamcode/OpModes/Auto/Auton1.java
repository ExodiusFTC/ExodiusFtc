package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.OpModes.Auto.paths.Paths1;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.TurretState;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@Config
@NextAutonomous(name = "Red Auto 1")
public class Auton1 extends NextOpMode {
    private final Robot robot;
    private final Paths1 paths = new Paths1();
    public static Pose startingPose = new Pose(55, 6.7, Math.toRadians(180));


    public static double targetVel = 580;
    public static double targetVel2 = -580;
    public static Pose botPose;


    public Auton1(Robot robot) {
        super(robot);
        this.robot = robot;
        telemetry.setMsTransmissionInterval(10);
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
        robot.getIntake().resetEncoder();
        robot.getPedro().follower().setPose(startingPose);
        robot.getBlocker().block().schedule();
        robot.intakeUp().schedule();
        Scheduler.reset();
    }

    private Command autoRoutine(Follower follower) {
        return sequential(
                robot.getBlocker().unblock(),
                waitMs(1500),
                robot.runIntake(),
                waitMs(1000),
                robot.getBlocker().block(),
                follow(follower, paths.firstCycle()),
                waitMs(500),
                follow(follower, paths.firstShootForSecondTip()),
                waitMs(800),
                robot.getBlocker().unblock(),
                waitMs(600),
                robot.intakeDown(),
                waitMs(100).with(robot.getBlocker().block()),
                follow(follower, paths.getFlowerOne()),
                waitMs(1000),
                follow(follower, paths.Back1()),
                follow(follower, paths.Forth1()),
                waitMs(1000),
                follow(follower, paths.Back2()),
                waitMs(1000),
                robot.getBlocker().unblock(),
                waitMs(500),
                robot.getBlocker().block().with(follow(follower, paths.moveToFlower2())).with(robot.intakeUp()),
                robot.intakeDown(),
                waitMs(500),
                follow(follower, paths.getFlower2()),
                waitMs(1000),
                follow(follower, paths.Back3()),
                follow(follower, paths.Forth3()),
                follow(follower, paths.shootAgain()),
                waitMs(500),
                robot.getBlocker().unblock(),
                waitMs(600),
                robot.getBlocker().block().with(robot.intakeUp()),
                follow(follower, paths.park())

                // Add mechanism commands here.
        );
    }
    @Override
    public void disabledPeriodic(){
        // Use the follower pose directly: the static botPose is still null until periodic() runs.
        Pose pose = robot.getPedro().follower().pose();
        robot.getTurretServos().setCurrentPos(robot.getIntake().getActualAngle());
        Telemetry.log("Actual angle", robot.getIntake().getActualAngle());
        Telemetry.log("Target Angle", robot.getTurretServos().calculateAimAngle(pose.x(), pose.y(), pose.heading()));
    }
    @Override
    public void start() {
        PoseStorage.saveAlliance(true);

        robot.getShooter().setTargetVelocity(targetVel);
        robot.getShooter().setTargetVelocity2(targetVel2);

        Follower follower = robot.getPedro().follower();

        follower.setPose(paths.startPose());

        autoRoutine(follower).schedule();
    }

    @Override
    public void periodic() {
        botPose = robot.getPedro().follower().pose();
        PoseStorage.savePose(botPose);   // always holds the latest pose for teleop

        robot.getShooter().setTargetVelocity(targetVel);
        robot.getShooter().setTargetVelocity2(targetVel2);

        // Turret auto-aim: feed in the current angle first so the aim calc uses this loop's reading.
        double actualAngle = robot.getIntake().getActualAngle();
        TurretState.save(actualAngle);   // keep latest angle stored every loop (same as the pose)
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
        // pose and turret angle are both already saved every loop in periodic()
    }
}