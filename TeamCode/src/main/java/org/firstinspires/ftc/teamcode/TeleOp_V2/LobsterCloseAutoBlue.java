package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LaserSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubHood;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubRamp;
import org.firstinspires.ftc.teamcode.subsystems.SubServoTurret;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import org.firstinspires.ftc.teamcode.subsystems.SubTurret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import static org.firstinspires.ftc.teamcode.TeleOp_V2.LobsterTele.BLUEGOAL;

@Autonomous(name = "LobsterCloseAutoBlue")
public class LobsterCloseAutoBlue extends NextFTCOpMode {
    private LaserSubsystem laser;


    public LobsterCloseAutoBlue(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubServoTurret.INSTANCE, SubHood.INSTANCE, SubRamp.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    public final Pose startPose = new Pose(34, 133, Math.toRadians(270));
    public final Pose MoveForPreload = new Pose(24, 104, Math.toRadians(270));
    public final Pose FirstStackPickup = new Pose(23, 80, Math.toRadians(270));
    public final Pose ShootFirstStack = new Pose(52, 78, Math.toRadians(180));
    public final Pose SecondStackPickup = new Pose(16, 63, Math.toRadians(180));
    public final Pose ShootSecondStack = new Pose(57, 79, Math.toRadians(180));
    public final Pose GateIntake = new Pose(10, 57, Math.toRadians(150));
    public final Pose GateIntakeReturn1 = new Pose(57, 79, Math.toRadians(180));

    private PathChain chain1;
    private PathChain chain2;
    private PathChain chain3;
    private PathChain chain4;
    private PathChain chain5;
    private PathChain chain6;
    private PathChain chain7;
    ElapsedTime elapsedTime = new ElapsedTime();


    public void buildPaths(){

        chain1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, MoveForPreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), MoveForPreload.getHeading())
                .setTranslationalConstraint(2.0)
                .setHeadingConstraint(Math.toRadians(3))
                .setTimeoutConstraint(1000)
                .build();

        chain2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(MoveForPreload, FirstStackPickup))
                .setConstantHeadingInterpolation(FirstStackPickup.getHeading())
                .setTranslationalConstraint(2.0)
                .setHeadingConstraint(Math.toRadians(3))
                .setTimeoutConstraint(1000)
                .build();

        chain3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(FirstStackPickup, ShootFirstStack))
                .setLinearHeadingInterpolation(FirstStackPickup.getHeading(), ShootFirstStack.getHeading())
                .setTranslationalConstraint(2.0)
                .setHeadingConstraint(Math.toRadians(3))
                .setTimeoutConstraint(1000)
                .build();

        chain4 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ShootFirstStack, new Pose(45, 57, Math.toRadians(180)), SecondStackPickup))
                .setConstantHeadingInterpolation(SecondStackPickup.getHeading())
                .setTimeoutConstraint(500)
                .build();

        chain5 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(SecondStackPickup, ShootSecondStack))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTimeoutConstraint(500)
                .build();

        chain6 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(ShootSecondStack, GateIntake))
                .setLinearHeadingInterpolation(ShootSecondStack.getHeading(), GateIntake.getHeading())
                .setTimeoutConstraint(500)
                .build();

        chain7 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(GateIntake, GateIntakeReturn1))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTimeoutConstraint(500)
                .build();
    }
    private Command shootingsequence(){
        return new SequentialGroup(
                new Delay(0.2),
                SubIntake.INSTANCE.stopTransfer.and(SubIntake.INSTANCE.StopIntake),
                SubRamp.INSTANCE.RampUp,
                SubIntake.INSTANCE.HoldIntake.and(SubIntake.INSTANCE.transferIntake),
                new Delay(0.4)
        );
    }
    /*private Command GateIntaking(){
        return new ParallelRaceGroup(
                new WaitUntil(() -> {
                        boolean balls = laser.threeBalls();
                        return balls;
        }),
                new Delay(1.5)
        );*/

   /* private Command GateIntaking() {
        return new ParallelRaceGroup(
                new WaitUntil(() -> laser.threeBalls()),
                new Delay(1.5)
        );
    }*/

    private Command GateIntaking() {
        return new ParallelRaceGroup(
                new Delay(1.5)
        );
    }


    private Command autonomousRoutine(){
        return new SequentialGroup(
                SubIntake.INSTANCE.HoldIntake.and(SubIntake.INSTANCE.transferIntake),
                new FollowPath(chain1),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.5),
                SubRamp.INSTANCE.RampDown,
                new FollowPath(chain2),
                new FollowPath(chain3),
                shootingsequence(),
                SubIntake.INSTANCE.slowTransfer,
                new FollowPath(chain4).and(SubRamp.INSTANCE.RampDown),
                new Delay(0.3),
                new FollowPath(chain5),
                shootingsequence(),
                SubIntake.INSTANCE.slowTransfer,
                new FollowPath(chain6).and(SubRamp.INSTANCE.RampDown),
                new Delay(2),
                //new WaitUntil(() -> laser.threeBalls()).raceWith(new Delay(1.5)),
                new FollowPath(chain7),
                new Delay(0.2).and(SubIntake.INSTANCE.transferIntake),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.4),
                SubIntake.INSTANCE.slowTransfer,
                new FollowPath(chain6).and(SubRamp.INSTANCE.RampDown),
                new Delay(2),
                //new WaitUntil(() -> laser.threeBalls()).raceWith(new Delay(1.5)),
                new FollowPath(chain7),
                new Delay(0.2).and(SubIntake.INSTANCE.transferIntake),
                SubRamp.INSTANCE.RampUp,
                new Delay(0.4),
                SubIntake.INSTANCE.slowTransfer,
                new FollowPath(chain6).and(SubRamp.INSTANCE.RampDown),
                new Delay(2),
                //new WaitUntil(() -> laser.threeBalls()).raceWith(new Delay(1.5)),
                new FollowPath(chain7),
                new Delay(0.2).and(SubIntake.INSTANCE.transferIntake),
                SubRamp.INSTANCE.RampUp

        );
    }

    private Command Initialize(){
        return new SequentialGroup(
                SubIntake.INSTANCE.HoldIntake,
                SubIntake.INSTANCE.StopIntake,
                SubIntake.INSTANCE.transferIntake,
                SubIntake.INSTANCE.stopTransfer,
                SubRamp.INSTANCE.RampUp,
                SubRamp.INSTANCE.RampDown,
                SubShoot.INSTANCE.StopShoot
        );
    }

    @Override
    public void onInit(){
        laser = new LaserSubsystem(hardwareMap);
        SubServoTurret.INSTANCE.initlut();
        SubShoot.INSTANCE.setPIDTRUE(false);
        SubHood.INSTANCE.initLut();
        SubShoot.INSTANCE.initlut();
        Initialize().schedule();
        PedroComponent.follower().setPose(startPose);
        elapsedTime.reset();
    }

    @Override
    public void onStartButtonPressed() {
        SubShoot.INSTANCE.setPIDTRUE(true);
        buildPaths();
        PedroComponent.follower().update();
        autonomousRoutine().schedule();
        elapsedTime.startTime();
    }

    @Override
    public void onUpdate(){
        laser.update();
        boolean threeballs = laser.threeBalls();
        SubShoot.INSTANCE.setPIDTRUE(true);
        PedroComponent.follower().update();
        double distFromGoal = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
        double hoodtune = SubHood.INSTANCE.getHoodlut(distFromGoal);
        SubHood.INSTANCE.sethoodtune(hoodtune);
        SubHood.INSTANCE.HoodInterpolation().schedule();
        double shootertune = SubShoot.INSTANCE.getlutVel(distFromGoal);
        SubShoot.INSTANCE.setTargetvelocity(shootertune);
        SubShoot.INSTANCE.InterpolationTuning().schedule();
        double despos = SubServoTurret.INSTANCE.calculate(PedroComponent.follower().getPose());
        if (elapsedTime.time()>=1.5){
            SubServoTurret.INSTANCE.setPos(despos);
        }
        else {
            SubServoTurret.INSTANCE.setPos(0.5);
        }
        telemetry.addData("Robot Pos", PedroComponent.follower().getPose().toString());
        telemetry.addData("threeballs", threeballs);
        telemetry.update();
    }

}