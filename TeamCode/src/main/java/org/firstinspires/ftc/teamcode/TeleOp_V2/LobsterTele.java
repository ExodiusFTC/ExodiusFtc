package org.firstinspires.ftc.teamcode.TeleOp_V2;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SubHood;
import org.firstinspires.ftc.teamcode.subsystems.SubShoot;


import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "LobsterTele")
public class LobsterTele extends NextFTCOpMode{
    public LobsterTele() {
        addComponents(
                new SubsystemComponent(SubShoot.INSTANCE, SubHood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    public double shootertune = 1000;
    public double hoodtune = 0;
    @Override
    public void onInit(){}

    @Override
    public void onStartButtonPressed(){}

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
