package org.firstinspires.ftc.teamcode.TeleOp_V2;

import static org.firstinspires.ftc.teamcode.TeleOp_V2.LobsterTele.BLUEGOAL;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LaserSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubHood;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubRamp;
import org.firstinspires.ftc.teamcode.subsystems.SubServoTurret;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "LobsterFarAutoBlue")
public class LobsterFarAutoBlue extends NextFTCOpMode {
    private ArtifactVision artifactVision;
    private LaserSubsystem laser;



    public LobsterFarAutoBlue(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubServoTurret.INSTANCE, SubHood.INSTANCE, SubRamp.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    public final Pose startPose = new Pose(56, 9, Math.toRadians(90));
    public final Pose firstShot = new Pose(44, 14, Math.toRadians(90));
    public final Pose ThirdStack_PickUP = new Pose(10, 34, Math.toRadians(180));
    public final Pose BackFromThirdStack = new Pose(47, 16, Math.toRadians(180));
    public final Pose HumanPlayer_PickUp = new Pose(9.5, 9, Math.toRadians(180));
    public final Pose BackFromHumanPlayer = new Pose(45, 12, Math.toRadians(180));
    private final Pose fallback = new Pose(14,40,Math.toRadians(90));


    private PathChain chain1;
    private PathChain chain35;
    private PathChain chain2;
    private PathChain chain3;
    private PathChain chain4;

    public void buildPaths(){

        chain1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(firstShot, new Pose(56, 32), ThirdStack_PickUP))
                .setLinearHeadingInterpolation(startPose.getHeading(), ThirdStack_PickUP.getHeading())
                .build();
        chain35 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, firstShot))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .setTimeoutConstraint(1000)
                .build();


        chain2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(ThirdStack_PickUP, BackFromThirdStack))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        chain3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(BackFromThirdStack, HumanPlayer_PickUp))
                .setConstantHeadingInterpolation(HumanPlayer_PickUp.getHeading())
                .build();

        chain4 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(HumanPlayer_PickUp, BackFromHumanPlayer))
                .setLinearHeadingInterpolation(HumanPlayer_PickUp.getHeading(), BackFromHumanPlayer.getHeading())
                .build();
    }
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
                    targetPose = new Pose(7.5, acct, targetHeading);
                    if(tx > 12){
                        Path dynamicPath = new Path(new BezierCurve(currentPose,new Pose(60, 40), targetPose));
                        dynamicPath.setLinearHeadingInterpolation(currentPose.getHeading(), targetHeading);

                        followCmd = new FollowPath(dynamicPath);

                    }
                    else {
                        Path dynamicPath = new Path(new BezierLine(currentPose, targetPose));
                        dynamicPath.setLinearHeadingInterpolation(currentPose.getHeading(), targetHeading);
                        followCmd = new FollowPath(dynamicPath);

                    }

                } else {
                    Path dynamicPath = new Path(new BezierCurve(currentPose, new Pose(10.5,7), fallback));
                    dynamicPath.setLinearHeadingInterpolation(currentPose.getHeading(), fallback.getHeading());
                    followCmd = new FollowPath(dynamicPath);
                }
                followCmd.start();
            }
            @Override
            public void update() {
                if (laser.threeBalls()){
                    followCmd.stop(true);
                }
                else {
                    followCmd.update();
                }

            }
            @Override
            public boolean isDone() {
                return followCmd.isDone();
            }
            @Override
            public void stop(boolean interrupted) {
                followCmd.stop(interrupted);
            }
        };
    }
    private Command returnPathing() {
        return new Command() {
            private FollowPath returning;
            @Override
            public void start() {
                Pose currentPose = PedroComponent.follower().getPose();
                Path dynamicPath = new Path(new BezierLine(currentPose,BackFromHumanPlayer));
                dynamicPath.setLinearHeadingInterpolation(currentPose.getHeading(), BackFromHumanPlayer.getHeading());
                returning= new FollowPath(dynamicPath);
                returning.start();
            }
            @Override
            public void update() {
                returning.update();
            }
            @Override
            public boolean isDone() {
                return returning.isDone();
            }
            @Override
            public void stop(boolean interrupted) {
                returning.stop(interrupted);
            }
        };
    }


    private Command autonomousRoutine(){
        return new SequentialGroup(
                SubRamp.INSTANCE.RampDown.and(SubIntake.INSTANCE.HoldIntake.and(SubIntake.INSTANCE.transferIntake)),
                new FollowPath(chain35),
                new WaitUntil(() -> (Math.abs(SubShoot.INSTANCE.getvel() - SubShoot.INSTANCE.getlutVel(startPose.distanceFrom(BLUEGOAL)))) <= 50),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.5),
                SubRamp.INSTANCE.RampDown.and(SubIntake.INSTANCE.slowTransfer),
                new FollowPath(chain1),
                new FollowPath(chain2),
                new Delay(0.2).and(SubIntake.INSTANCE.transferIntake),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.4),
                SubRamp.INSTANCE.RampDown,
                new FollowPath(chain3).and(SubIntake.INSTANCE.slowTransfer),
                new Delay(0.3),
                new FollowPath(chain4).and(SubIntake.INSTANCE.transferIntake),
                new Delay(0.1),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.4),
                SubIntake.INSTANCE.slowTransfer,
                driveToDetectedBallOrFallback().and(SubRamp.INSTANCE.RampDown),
                new Delay(1),
                returnPathing().and(SubIntake.INSTANCE.transferIntake),
                new Delay(0.3),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.4),
                SubIntake.INSTANCE.slowTransfer,
                driveToDetectedBallOrFallback().and(SubRamp.INSTANCE.RampDown),
                new Delay(1),
                returnPathing().and(SubIntake.INSTANCE.transferIntake),
                new Delay(0.3),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.4),
                SubIntake.INSTANCE.slowTransfer,
                driveToDetectedBallOrFallback().and(SubRamp.INSTANCE.RampDown),
                new Delay(1),
                returnPathing().and(SubIntake.INSTANCE.transferIntake),
                new Delay(0.3),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.4),
                SubIntake.INSTANCE.slowTransfer,
                driveToDetectedBallOrFallback().and(SubRamp.INSTANCE.RampDown),
                new Delay(1),
                returnPathing().and(SubIntake.INSTANCE.transferIntake),
                new Delay(0.3),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.4)
        );
    }



    private Command Initialize(){
        return new SequentialGroup(
                SubIntake.INSTANCE.HoldIntake,
                SubIntake.INSTANCE.StopIntake,
                SubIntake.INSTANCE.stopTransfer,
                SubRamp.INSTANCE.RampUp,
                SubRamp.INSTANCE.RampDown,
                SubShoot.INSTANCE.StopShoot
        );
    }

    @Override
    public void onInit(){
        laser = new LaserSubsystem(hardwareMap);
        artifactVision = new ArtifactVision(hardwareMap, 0);
        SubShoot.INSTANCE.setPIDTRUE(false);
        SubServoTurret.INSTANCE.initlut();
        SubHood.INSTANCE.initLut();
        SubShoot.INSTANCE.initlut();
        Initialize().schedule();
        PedroComponent.follower().setPose(startPose);
    }

    @Override
    public void onStartButtonPressed() {
        buildPaths();
        PedroComponent.follower().update();
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate(){
        SubServoTurret.INSTANCE.initlut();
        laser.update();
        boolean threeballs = laser.threeBalls();
        artifactVision.update();
        SubShoot.INSTANCE.setPIDTRUE(true);
        PedroComponent.follower().update();
        double distFromGoal = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
        double hoodtune = SubHood.INSTANCE.getHoodlut(distFromGoal);
        SubHood.INSTANCE.sethoodtune(hoodtune);
        SubHood.INSTANCE.HoodInterpolation().schedule();
        double shootertune = SubShoot.INSTANCE.getlutVel(distFromGoal);
        SubShoot.INSTANCE.setTargetvelocity(shootertune);
        SubShoot.INSTANCE.InterpolationTuning().schedule();
        double despos = SubServoTurret.INSTANCE.calculateBlueFar(PedroComponent.follower().getPose());
        SubServoTurret.INSTANCE.setPos(despos);


        telemetry.addData("Robot Pos", PedroComponent.follower().getPose().toString());
        telemetry.addData("targ vel", shootertune);
        telemetry.addData("shooter vel", SubShoot.INSTANCE.getvel());
        telemetry.update();
    }

}