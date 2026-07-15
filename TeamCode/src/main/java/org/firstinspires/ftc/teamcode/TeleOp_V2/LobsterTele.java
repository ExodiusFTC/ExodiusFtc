package org.firstinspires.ftc.teamcode.TeleOp_V2;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SubHood;
import org.firstinspires.ftc.teamcode.subsystems.SubIntake;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;


import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@TeleOp(name = "LobsterTele")
public class LobsterTele extends NextFTCOpMode{
    public LobsterTele() {
        addComponents(
                new SubsystemComponent(SubShoot.INSTANCE, SubHood.INSTANCE, SubIntake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    public double shootertune = 0;
    public double hoodtune = 0.5;
    @Override
    public void onInit(){}

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
    }

    @Override
    public void onUpdate(){
        if (gamepad2.aWasPressed()){
            shootertune += 50;
        }
        if (gamepad2.bWasPressed()){
            shootertune -= 50;
        }
        if (gamepad2.leftBumperWasPressed()){
            hoodtune +=0.05;
        }
        if (gamepad2.rightBumperWasPressed()){
            hoodtune -=0.05;
        }
        SubHood.INSTANCE.sethoodtune(hoodtune);
        SubShoot.INSTANCE.setTargetvelocity(shootertune);
        SubHood.INSTANCE.HoodInterpolation().schedule();



        if (gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(true);
            SubShoot.INSTANCE.InterpolationTuning().schedule();
        } else if (!gamepad2.x){
            SubShoot.INSTANCE.setPIDTRUE(false);
        }
        telemetry.addData("flywheelvel", SubShoot.INSTANCE.getvel());
        telemetry.addData("Hood Pos", SubHood.INSTANCE.getHoodtune());
        telemetry.addData("target velocity", SubShoot.INSTANCE.getTargetvelocity());
        telemetry.update();
    }

}
