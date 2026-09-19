package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.triggers.CommandGamepad;

public class Robot implements NextRobot {

    private final Intake intake = new Intake();
    public Robot() {}
    public Intake getIntake(){
        return intake;
    }
    public Command runIntake(){
        return intake.run();
    }
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake);
    }



    @Override
    public void periodic(){


    }
}
