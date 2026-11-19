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
        //robot.getIntake().resetEncoder();
        robot.getPedro().follower().setPose(startingPose);
        Scheduler.reset();
    }

    private Command autoRoutine(Follower follower) {
        return sequential(
                waitMs(3000),
                robot.getIntake().run(),
                waitMs(1000),
                follow(follower, paths.firstCycle())
                // Add mechanism commands here.
        );
    }
    @Override
    public void disabledPeriodic(){
        robot.getPedro().follower().setPose(startingPose);

        robot.intakeUp().schedule();
        //robot.getIntake().resetEncoder();
        if(gamepad2.xWasPressed()){
            robot.getIntake().resetEncoder();
        }
        Telemetry.log("Actual angle", robot.getIntake().getActualAngle());
        Telemetry.log("Target Angle", robot.getTurretServos().calculateAimAngle(botPose.x(), botPose.y(), botPose.heading()));



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
//        robot.getTurretServos().setTargetPos(robot.getTurretServos().calculateAimAngle(botPose.x(), botPose.y(), botPose.heading()));
//        robot.getTurretServos().setClosedLoop(true);
        Telemetry.log("Actual angle", robot.getIntake().getActualAngle());
        Telemetry.log("Target Angle", robot.getTurretServos().calculateAimAngle(botPose.x(), botPose.y(), botPose.heading()));

    }

    @Override
    public void end() {
    }
}