package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.triggers.CommandGamepad;

public class Robot implements NextRobot {

    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    public Robot() {}
    public Intake getIntake(){
        return intake;
    }
    public Shooter getShooter(){
        return shooter;
    }
    public Command runIntake(){
        return intake.run();
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, shooter);
    }



    @Override
    public void periodic(){


    }
}
