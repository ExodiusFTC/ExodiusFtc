package org.firstinspires.ftc.teamcode.TeleOp_V2;



import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.LaserSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubHood;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubRamp;
import org.firstinspires.ftc.teamcode.subsystems.SubServoTurret;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LobsterTele")
public class LobsterTele extends NextFTCOpMode{

    private LaserSubsystem laser;
    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.

    public LobsterTele() {
        addComponents(
                new SubsystemComponent(SubShoot.INSTANCE, SubHood.INSTANCE, SubIntake.INSTANCE, SubRamp.INSTANCE, SubServoTurret.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    public static Pose startingPose = new Pose(9, 9, Math.toRadians(180));
    public static Pose BLUEGOAL = new Pose(0, 144, Math.toRadians(0));


    double robotHeading;
    public double shootertune = 0;
    public double hoodtune = 0.5;
    public double ramptune = 0.5;
    public boolean detected;
    public double turrettune = 0.502;
    Command transfer = new ParallelGroup(
            SubIntake.INSTANCE.HoldIntake,
            SubIntake.INSTANCE.transferIntake,
            SubRamp.INSTANCE.RampUp
    );
    Command stoppingTransfer = new ParallelGroup(
            SubIntake.INSTANCE.StopIntake,
            SubIntake.INSTANCE.stopTransfer,
            SubRamp.INSTANCE.RampDown
    );

    @Override
    public void onInit(){
        laser = new LaserSubsystem(hardwareMap);
        SubServoTurret.INSTANCE.initlut();
        SubHood.INSTANCE.initLut();
        SubShoot.INSTANCE.initlut();
        PedroComponent.follower().setStartingPose(startingPose);
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 1000)  //  Rumble right motor 100% for 1000 mSec
                .addStep(0.0, 0.0, 1000)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 1000)  //  Rumble left motor 100% for 1000 mSec
                .addStep(1.0, 1.0, 1000) // run both for 1000 milliseconds
                .build();
    }


    @Override
    public void onStartButtonPressed(){
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(SubIntake.INSTANCE.HoldIntake.and(SubIntake.INSTANCE.slowTransfer))
                .whenBecomesFalse(SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.stopTransfer));
        Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(transfer)
                .whenBecomesFalse(stoppingTransfer);
//        Gamepads.gamepad2().leftBumper()
//                .whenBecomesTrue(SubServoTurret.INSTANCE.testing)
//                .whenBecomesFalse(SubServoTurret.INSTANCE.middle);
//        Gamepads.gamepad2().rightBumper()
//                .whenBecomesTrue(SubServoTurret.INSTANCE.testing2)
//                .whenBecomesFalse(SubServoTurret.INSTANCE.middle);



    }

    @Override
    public void onUpdate(){
        laser.update();
        boolean threeballs = laser.threeBalls();
        double distFromGoal = PedroComponent.follower().getPose().distanceFrom(BLUEGOAL);
        PedroComponent.follower().update();

        SubServoTurret.INSTANCE.setPos(turrettune);
        //gamepad1.runRumbleEffect(customRumbleEffect);




        if (gamepad1.dpadUpWasPressed()){
            PedroComponent.follower().setPose(startingPose);
        }
        hoodtune = SubHood.INSTANCE.getHoodlut(distFromGoal);
        SubHood.INSTANCE.sethoodtune(hoodtune);
        SubHood.INSTANCE.HoodInterpolation().schedule();
        shootertune = SubShoot.INSTANCE.getlutVel(distFromGoal);
        SubShoot.INSTANCE.setTargetvelocity(shootertune);
        double despos = SubServoTurret.INSTANCE.calculate(PedroComponent.follower().getPose());
        if(gamepad2.a){
            SubServoTurret.INSTANCE.setPos(despos);
        }





        if (gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(true);
            SubShoot.INSTANCE.InterpolationTuning().schedule();
        } else if (!gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(false);
        }

        telemetry.addData("dist from goal", PedroComponent.follower().getPose().distanceFrom(BLUEGOAL));
        telemetry.addData("botpos", PedroComponent.follower().getPose().toString());
        telemetry.addData("turret1:", SubServoTurret.INSTANCE.getPos1());
        telemetry.addData("turret2:", SubServoTurret.INSTANCE.getPos2());
        telemetry.addData("Laser Beam State", laser.getDetection());
        telemetry.addData("Three Balls", threeballs);
        telemetry.addData("flywheelvel", SubShoot.INSTANCE.getvel());
        telemetry.addData("Hood Pos", SubHood.INSTANCE.getHoodtune());
        telemetry.addData("target velocity", SubShoot.INSTANCE.getTargetvelocity());
        telemetry.update();
    }


}
