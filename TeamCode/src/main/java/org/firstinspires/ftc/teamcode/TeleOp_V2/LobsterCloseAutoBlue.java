package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubHood;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubRamp;
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

@Autonomous(name = "LobsterCloseAutoBlue")
public class LobsterCloseAutoBlue extends NextFTCOpMode {

    public LobsterCloseAutoBlue(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubServoTurret.INSTANCE, SubHood.INSTANCE, SubRamp.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    public final Pose startPose = new Pose(34, 133, Math.toRadians(270));
    public final Pose MoveForPreload = new Pose(24, 104, Math.toRadians(270));
    public final Pose FirstStackPickup = new Pose(24, 85, Math.toRadians(270));
    public final Pose ShootFirstStack = new Pose(52, 78, Math.toRadians(180));
    public final Pose SecondStackPickup = new Pose(15, 62, Math.toRadians(180));
    public final Pose ShootSecondStack = new Pose(57, 79, Math.toRadians(180));
    public final Pose GateIntake = new Pose(12, 58, Math.toRadians(145));
    public final Pose GateIntakeReturn1 = new Pose(57, 79, Math.toRadians(180));

    private PathChain chain1;
    private PathChain chain2;
    private PathChain chain3;
    private PathChain chain4;
    private PathChain chain5;
    private PathChain chain6;
    private PathChain chain7;

    public void buildPaths(){

        chain1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, MoveForPreload))
                .setLinearHeadingInterpolation(startPose.getHeading(), MoveForPreload.getHeading())
                .setTranslationalConstraint(2.0)
                .setHeadingConstraint(Math.toRadians(3))
                .setTimeoutConstraint(3000)
                .build();

        chain2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(MoveForPreload, FirstStackPickup))
                .setConstantHeadingInterpolation(FirstStackPickup.getHeading())
                .setTranslationalConstraint(2.0)
                .setHeadingConstraint(Math.toRadians(3))
                .setTimeoutConstraint(3000)
                .build();

        chain3 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(FirstStackPickup, ShootFirstStack))
                .setLinearHeadingInterpolation(FirstStackPickup.getHeading(), ShootFirstStack.getHeading())
                .setTranslationalConstraint(2.0)
                .setHeadingConstraint(Math.toRadians(3))
                .setTimeoutConstraint(3000)
                .build();

        chain4 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(ShootFirstStack, new Pose(45, 57, Math.toRadians(180)), SecondStackPickup))
                .setConstantHeadingInterpolation(SecondStackPickup.getHeading())
                .build();

        chain5 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(SecondStackPickup, ShootSecondStack))
                .setLinearHeadingInterpolation(SecondStackPickup.getHeading(), ShootSecondStack.getHeading())
                .build();

        chain6 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(ShootSecondStack, GateIntake))
                .setLinearHeadingInterpolation(ShootSecondStack.getHeading(), GateIntake.getHeading())
                .build();

        chain7 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(GateIntake, GateIntakeReturn1))
                .setLinearHeadingInterpolation(GateIntake.getHeading(), GateIntakeReturn1.getHeading())
                .build();
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
                new Delay(0.2),
                SubRamp.INSTANCE.RampUp
//                new FollowPath(chain4),
//                new FollowPath(chain5),
//                new FollowPath(chain6),
//                new FollowPath(chain7)
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
        Initialize().schedule();
        PedroComponent.follower().setPose(startPose);
    }

    @Override
    public void onStartButtonPressed() {
        SubShoot.INSTANCE.setPIDTRUE(true);
        buildPaths();
        PedroComponent.follower().update();
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate(){
        PedroComponent.follower().update();
        double despos = SubServoTurret.INSTANCE.calculate(PedroComponent.follower().getPose());
        SubServoTurret.INSTANCE.setPos(despos);
        telemetry.addData("Robot Pos", PedroComponent.follower().getPose().toString());
        telemetry.update();
    }

}