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


    private LaserSubsystem Laser;
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
        SubShoot.INSTANCE.initlut();
        SubHood.INSTANCE.initLut();
        laser = new LaserSubsystem(hardwareMap);
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
                .whenBecomesTrue(SubIntake.INSTANCE.HoldIntake.and(SubIntake.INSTANCE.transferIntake))
                .whenBecomesFalse(SubIntake.INSTANCE.StopIntake.and(SubIntake.INSTANCE.stopTransfer));
        Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(SubRamp.INSTANCE.RampUp)
                .whenBecomesFalse(SubRamp.INSTANCE.RampDown);
//        Gamepads.gamepad2().leftBumper()
//                .whenBecomesTrue(SubServoTurret.INSTANCE.testing)
//                .whenBecomesFalse(SubServoTurret.INSTANCE.middle);
//        Gamepads.gamepad2().rightBumper()
//                .whenBecomesTrue(SubServoTurret.INSTANCE.testing2)
//                .whenBecomesFalse(SubServoTurret.INSTANCE.middle);



    }

    @Override
    public void onUpdate(){
        PedroComponent.follower().update();
        shootertune  = SubShoot.INSTANCE.getlutVel(PedroComponent.follower().getPose().distanceFrom(BLUEGOAL));
        hoodtune = SubHood.INSTANCE.getHoodlut(PedroComponent.follower().getPose().distanceFrom(BLUEGOAL));

        if (gamepad2.dpadDownWasPressed()){
            turrettune += 0.05;
        }
        if(gamepad2.dpadUpWasPressed()){
            turrettune-=0.05;
        }
        SubServoTurret.INSTANCE.setPos(turrettune);
        //gamepad1.runRumbleEffect(customRumbleEffect);
        laser.update();
        //double pos = SubServoTurret.INSTANCE.calculate(example);
        //SubServoTurret.INSTANCE.setPos(pos);

//        if (gamepad2.aWasPressed()){
//            shootertune += 50;
//        }
//        if (gamepad2.bWasPressed()){
//            shootertune -= 50;
//        }

//        if (gamepad1.leftBumperWasPressed()){
//            hoodtune +=0.05;
//        }
//        if (gamepad1.rightBumperWasPressed()){
//            hoodtune -=0.05;
//        }
        if (gamepad1.aWasPressed()){
            ramptune +=0.05;
        }
        if (gamepad1.bWasPressed()){
            ramptune -=0.05;
        }
        SubHood.INSTANCE.sethoodtune(hoodtune);
        SubHood.INSTANCE.HoodInterpolation().schedule();
        SubShoot.INSTANCE.setTargetvelocity(shootertune);
        double despos = SubServoTurret.INSTANCE.calculate(PedroComponent.follower().getPose());
        SubServoTurret.INSTANCE.setPos(despos);
        //SubRamp.INSTANCE.Ramptune(ramptune).schedule();

//        double dx = BLUEGOAL.getX() - PedroComponent.follower().getPose().getX();
//        double dy = BLUEGOAL.getY() - PedroComponent.follower().getPose().getY();
//        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
//        double robotHeading = Math.toDegrees(PedroComponent.follower().getHeading());
//        double turretTargetAngle = fieldAngleToGoal - robotHeading;
//        double CorrectTurning = normalizeAngle(turretTargetAngle);
//        double despos = 0.00201389*CorrectTurning+0.502333;
//        SubServoTurret.INSTANCE.setPos(despos);


        if (gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(true);
            SubShoot.INSTANCE.InterpolationTuning().schedule();
        } else if (!gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(false);
        }

        telemetry.addData("dist from goal", PedroComponent.follower().getPose().distanceFrom(BLUEGOAL));
        telemetry.addData("botpos", PedroComponent.follower().getPose().toString());
        //telemetry.addData("Laser Beam State", laser.getState() ? "DETECTED" : "CLEAR");
        telemetry.addData("flywheelvel", SubShoot.INSTANCE.getvel());
        telemetry.addData("Hood Pos", SubHood.INSTANCE.getHoodtune());
        telemetry.addData("target velocity", SubShoot.INSTANCE.getTargetvelocity());
        telemetry.update();
    }
//    double normalizeAngle(double angle) {
//        angle = -1 * (180 - angle);
//        while (angle > 180) angle -= 360;
//        while (angle < -180) angle += 360;
//        return angle;
//    }

}
