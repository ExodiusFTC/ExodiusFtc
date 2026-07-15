package org.firstinspires.ftc.teamcode.subsystems;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;


public class SubIntake implements Subsystem {
    public static final SubIntake INSTANCE = new SubIntake();
    private SubIntake(){}
    private MotorEx IntakeMotor = new MotorEx("I");
    private MotorEx TransferMotor = new MotorEx("Transfer");

    //384.5 encoder ticks per revolution
    //2787.625 max tps
    // max speed probably around 2500
    private ServoEx Blocker = new ServoEx("Kick");




    public Command KickUp = new SetPosition(Blocker, 0.97).requires(this);
    public Command KickDown = new SetPosition(Blocker, 0.45).requires(this);
    // Kickdown - opens blocker
    // KickUp - closes blocker
    public Command HoldIntake = new SetPower(IntakeMotor, 1).requires(this);
    public Command transferIntake = new SetPower(TransferMotor, -1).requires(this);
    public Command stopTransfer = new SetPower(TransferMotor, 0).requires(this);
    public Command StopIntake = new SetPower(IntakeMotor, 0).requires(this);
    public Command ReverseIntake = new SetPower(IntakeMotor, -1).requires(this);



    @Override
    public void periodic(){

    }


    public double getIntakeVel(){
        return IntakeMotor.getVelocity();
    }

}