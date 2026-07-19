package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class SubShoot implements Subsystem {
    public static final SubShoot INSTANCE = new SubShoot();
    private SubShoot(){}


    private MotorEx shooterMotor = new MotorEx("SH");
    private MotorEx shooterMotor2 = new MotorEx("SH2");
    private MotorGroup SHOOTERS = new MotorGroup(shooterMotor, shooterMotor2);
    public boolean PIDTRUE;
    double shottune;
    double hoodtune;

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.001, 0, 0)
            .basicFF(0.000345, 0, 0.05)
            .build();


    public Command StopShoot = new SetPower(shooterMotor, 0).requires(this);

    public Command PIDshot = new RunToVelocity(controlSystem, 1100, 30).requires(this);
    public Command PIDfarShot = new RunToVelocity(controlSystem, 1400, 30).requires(this);

    public Command InterpolationTuning(){
        return new RunToVelocity(controlSystem, shottune, 30 ).requires(this);
    }


    public double getvel(){
        return SHOOTERS.getVelocity();
    }
    public void setTargetvelocity(double targvel){

        shottune = targvel;
    }
    public double getTargetvelocity(){

        return shottune;
    }






    @Override
    public void initialize() {
        // initialization logic (runs on init)

        //shooterMotor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
        if (PIDTRUE){
            SHOOTERS.setPower(controlSystem.calculate(SHOOTERS.getState()));
            //shooterMotor.setPower(controlSystem.calculate(shooterMotor.getState()));
        }
        if (!PIDTRUE){
            SHOOTERS.setPower(0);
        }


    }
    public void setPIDTRUE(boolean pidstate){
        PIDTRUE = pidstate;
    }
    public boolean getPIDTRUE(){
        return PIDTRUE;
    }
}