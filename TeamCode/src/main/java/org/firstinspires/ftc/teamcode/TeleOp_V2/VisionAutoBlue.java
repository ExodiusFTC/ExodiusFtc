package org.firstinspires.ftc.teamcode.TeleOp_V2;

import static java.lang.Math.clamp;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SubHood;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubServoTurret;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "VisionAutoBlue")
public class VisionAutoBlue extends NextFTCOpMode {
    public VisionAutoBlue(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubServoTurret.INSTANCE, SubHood.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    double DISTANCETOBLUEGOAL;
    double shootertune;
    double HoodTune;

    public final Pose startPose = new Pose(56, 8, Math.toRadians(180));
    public static Pose BLUEGOAL = new Pose(4, 144, Math.toRadians(0));

    // fallback destination when no ball is seen at all
    private final Pose IntakeStack = new Pose(10, 35, Math.toRadians(180));
    private final Pose fallback = new Pose(14,40,Math.toRadians(90));
    private ArtifactVision artifactVision;

    public void buildPaths(){
        // add your real PathChains/Paths here for this auto's routine
    }

    // reads tx once when the command starts, then picks one of two fixed
    // pickup poses based on your calibration table; falls back to IntakeStack
    // if no ball is detected at all
    private Command driveToDetectedBallOrFallback() {
        return new Command() {
            private FollowPath followCmd;

            @Override
            public void start() {
                Pose currentPose = PedroComponent.follower().getPose();

                if (artifactVision.hasTarget()) {
                    double tx = artifactVision.getTx(); // read once, right now

                    double targetHeading = Math.toRadians(180);
                    Pose targetPose;
                    double targy = tx+9.2;
                    double acct = com.acmerobotics.roadrunner.Math.clamp(targy, 9.5, 80);
                    targetPose = new Pose(9, acct, targetHeading);
                    if(tx > 6){
                        Path dynamicPath = new Path(new BezierCurve(currentPose,new Pose(70, 45), targetPose));
                        dynamicPath.setLinearHeadingInterpolation(currentPose.getHeading(), targetHeading);

                        followCmd = new FollowPath(dynamicPath);

                    }
                    else {
                        Path dynamicPath = new Path(new BezierLine(currentPose, targetPose));
                        dynamicPath.setLinearHeadingInterpolation(currentPose.getHeading(), targetHeading);
                        followCmd = new FollowPath(dynamicPath);

                    }

//                    if (tx <= 16) {
//                        // matches table rows: tx = -8, 0, 13
//                        targetPose = new Pose(10, 10, targetHeading);
//                    } else {
//                        // matches table rows: tx = 19, 24
//                        targetPose = new Pose(10, 35, targetHeading);
//                    }



                } else {
                    Path dynamicPath = new Path(new BezierCurve(currentPose, new Pose(13,7), fallback));
                    dynamicPath.setLinearHeadingInterpolation(currentPose.getHeading(), fallback.getHeading());
                    followCmd = new FollowPath(dynamicPath);
                }

                followCmd.start();
            }

            @Override
            public void update() { followCmd.update(); }

            @Override
            public boolean isDone() { return followCmd.isDone(); }

            @Override
            public void stop(boolean interrupted) { followCmd.stop(interrupted); }
        };
    }

    private Command autonomousRoutine(){
        return new SequentialGroup(
                driveToDetectedBallOrFallback()
        );
    }

    private Command Initialize(){
        return new SequentialGroup(
                SubIntake.INSTANCE.HoldIntake,
                SubIntake.INSTANCE.StopIntake,
                SubShoot.INSTANCE.StopShoot
        );
    }

    @Override
    public void onInit(){
        PedroComponent.follower().setPose(startPose);
        artifactVision = new ArtifactVision(hardwareMap, 0); // set your real blob pipeline index
        Initialize().schedule();
    }

    @Override
    public void onWaitForStart() {
        SubShoot.INSTANCE.setPIDTRUE(false);
    }

    @Override
    public void onStartButtonPressed() {
        SubShoot.INSTANCE.setPIDTRUE(true);
        buildPaths();
        PedroComponent.follower().update();
        artifactVision.update();
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate(){
        artifactVision.update();
        SubShoot.INSTANCE.setPIDTRUE(true);
        SubShoot.INSTANCE.PIDfarShot.schedule();
        telemetry.addData("Hood Pos", SubHood.INSTANCE.getHoodtune());
        telemetry.addData("Flywheel vel", SubShoot.INSTANCE.getvel());
        telemetry.addData("Robot Pos", PedroComponent.follower().getPose().toString());
        telemetry.addData("vision tx", artifactVision.getTx());
        telemetry.addData("vision has target", artifactVision.hasTarget());
        DISTANCETOBLUEGOAL = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
        HoodTune = -0.00000594867 * Math.pow(DISTANCETOBLUEGOAL, 3) + 0.00178147 * Math.pow(DISTANCETOBLUEGOAL, 2) - 0.172839 * DISTANCETOBLUEGOAL + 5.77029;
        SubHood.INSTANCE.sethoodtune(HoodTune);
        SubHood.INSTANCE.HoodInterpolation().schedule();
        telemetry.update();
    }

    double normalizeAngle(double angle) {
        angle = -1 * (180 - angle);
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}