package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

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

    public static double targetVel = 520;
    public static double targetVel2 = 680;

    public Auton1(Robot robot) {
        super(robot);
        this.robot = robot;
        telemetry.setMsTransmissionInterval(10);
        Telemetry.addBackend(FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
    }

    private Command autoRoutine(Follower follower) {
        return sequential(
                waitMs(3000),
                robot.getIntake().run(),
                waitMs(1000),
                follow(follower, paths.path1())
                // Add mechanism commands here.
        );
    }

    @Override
    public void start() {
        // Get the follower from the Pedro mechanism that Robot already created.
        // This must happen here (not in the constructor / a field initializer):
        // the hardware map isn't available yet when the OpMode is constructed.
        Follower follower = robot.getPedro().follower();

        // Tell Pedro where the robot actually starts, so the path begins from there.
        follower.setPose(paths.startPose());

        autoRoutine(follower).schedule();
        robot.intakeUp().schedule(); // schedule once; calling it every loop did nothing
    }

    @Override
    public void periodic() {
        robot.getShooter().setTargetVelocity(targetVel);
        robot.getShooter().setTargetVelocity2(targetVel2);
    }

    @Override
    public void end() {
    }
}