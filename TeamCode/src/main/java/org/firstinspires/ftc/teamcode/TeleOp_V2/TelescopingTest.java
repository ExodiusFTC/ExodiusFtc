package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FeedbackServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SubCRServo;

import dev.nextftc.control.ControlSystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


@TeleOp(name = "Telescoping Test")
public class TelescopingTest extends NextFTCOpMode {
    private MotorEx boxTube = new MotorEx("BT");
    private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .basicFF(0.000004, 0, 0)
            .build();


    public TelescopingTest() {
        addComponents(
                new SubsystemComponent(SubCRServo.INSTANCE, FeedbackServoSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    public Command extend = new RunToPosition(controlSystem, 1700, 30).requires(this);
    public Command Hold = new RunToVelocity(controlSystem, 0, 30).requires(this);
    public Command go1 = new SetPower(boxTube, 1).requires(this);
    public Command stop  = new SetPower(boxTube, 0).requires(this);

    public Command go2 = new SetPower(boxTube, -1).requires(this);

    //384.5 ticks per revolution
    // MAXIMUM THEORETICAL: 2787
    // applicable max: 2300
    @Override
    public void onInit(){

    }
    @Override
    public void onStartButtonPressed(){
        Gamepads.gamepad1().a()
                .whenBecomesTrue(go1)
                .whenBecomesFalse(stop);
        Gamepads.gamepad1().b()
                .whenBecomesTrue(go2)
                .whenBecomesFalse(stop);
    }

    @Override
    public void onUpdate(){
        //boxTube.setPower(controlSystem.calculate(boxTube.getState()));
        telemetry.addData("Velocity", boxTube.getVelocity());


//        if (gamepad1.a){
//            boxTube.setPower(-1);
//        }
//        if (gamepad1.b){
//            boxTube.setPower(1);
//        }
//        else {
//            boxTube.setPower(0);
//        }
    }
}
