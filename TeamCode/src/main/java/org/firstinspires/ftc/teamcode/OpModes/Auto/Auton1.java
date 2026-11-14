package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


import org.firstinspires.ftc.teamcode.OpModes.Auto.paths.Paths1;
import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous(name = "Red Auto 1")
public class Auton1 extends NextOpMode {
    Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Robot robot;
    Paths1 paths = new Paths1();
    Command pause = waitMs(2000); // waits 500ms
    public static double targetVel = 520;

    public static double targetVel2 =680;


    public Auton1(Robot robot){
        super(robot);
        this.robot = robot;
        telemetry.setMsTransmissionInterval(10);
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
    }

    private Command autoRoutine() {
        return sequential(
                pause,
                robot.getIntake().run(),
                follow(follower, paths.path1())
                // Add mechanism commands here.

        );
    }
    @Override
    public void start(){
        autoRoutine().schedule();
    }

    @Override
    public void periodic(){
        robot.getShooter().setTargetVelocity(targetVel);
        robot.getShooter().setTargetVelocity2(targetVel2);
        robot.intakeUp();


    }

    @Override
    public void end(){

    }
}
