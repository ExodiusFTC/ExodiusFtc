package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubHood;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubRamp;
import org.firstinspires.ftc.teamcode.subsystems.SubServoTurret;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "LobsterFarAutoBlue")
public class LobsterFarAutoBlue extends NextFTCOpMode {

    public LobsterFarAutoBlue(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(SubShoot.INSTANCE, SubIntake.INSTANCE, SubServoTurret.INSTANCE, SubHood.INSTANCE, SubRamp.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    public final Pose startPose = new Pose(44, 9, Math.toRadians(90));
    public final Pose ThirdStack_PickUP = new Pose(23, 27, Math.toRadians(90));
    public final Pose BackFromThirdStack = new Pose(49, 10, Math.toRadians(180));
    public final Pose HumanPlayer_PickUp = new Pose(8, 9, Math.toRadians(180));
    public final Pose BackFromHumanPlayer = new Pose(45, 10, Math.toRadians(180));

    private Path path1;
    private Path path2;
    private Path path3;
    private Path path4;

    public void buildPaths(){

        path1 = new Path(new BezierLine(startPose, ThirdStack_PickUP));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), ThirdStack_PickUP.getHeading());

        path2 = new Path(new BezierLine(ThirdStack_PickUP, BackFromThirdStack));
        path2.setLinearHeadingInterpolation(ThirdStack_PickUP.getHeading(), BackFromThirdStack.getHeading());

        path3 = new Path(new BezierLine(BackFromThirdStack, HumanPlayer_PickUp));
        path3.setLinearHeadingInterpolation(BackFromThirdStack.getHeading(), HumanPlayer_PickUp.getHeading());
        path3.getReversed();

        path4 = new Path(new BezierLine(HumanPlayer_PickUp, BackFromHumanPlayer));
        path4.setLinearHeadingInterpolation(HumanPlayer_PickUp.getHeading(), BackFromHumanPlayer.getHeading());
        path4.getReversed();
    }

    @Override
    public void onInit(){
        PedroComponent.follower().setPose(startPose);
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
    }

    @Override
    public void onUpdate(){
        PedroComponent.follower().update();
        telemetry.addData("Robot Pos", PedroComponent.follower().getPose().toString());
        telemetry.update();
    }

}