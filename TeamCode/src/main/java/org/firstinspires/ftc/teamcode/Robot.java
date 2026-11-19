package org.firstinspires.ftc.teamcode;



import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mechanisms.Blocker;
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.TriangleIntake;
import org.firstinspires.ftc.teamcode.mechanisms.TurretServos;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.drive.DriveCommands;
import dev.nextftc.v2.pedro.PedroMechanism;

public class Robot implements NextRobot {

    private final PedroMechanism pedro = new PedroMechanism(Constants::create);

    private final TurretServos turretServos = new TurretServos();
    private final TriangleIntake triangleIntake = new TriangleIntake();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    //private final ELCencoder encoder = new ELCencoder();
    private final Blocker blocker = new Blocker();

    //private final TurretEncoder turretEncoder = new TurretEncoder();


    public Robot() {}

    public TurretServos getTurretServos(){
        return turretServos;
    }
    public Intake getIntake(){
        return intake;
    }
    public Shooter getShooter(){
        return shooter;
    }
    public Blocker getBlocker(){
        return blocker;
    }
    public TriangleIntake getTriangleIntake(){
        return triangleIntake;
    }
    public PedroMechanism getPedro(){
        return pedro;
    }


    public Command runIntake(){
        return intake.run();
    }
    public Command intakeUp(){
        return triangleIntake.up();
    }
    public Command intakeDown(){
        return triangleIntake.down();
    }
    public Command Block(){
        return blocker.block();
    }
    public Command Unblock(){
        return blocker.unblock();
    }





    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, shooter, blocker, triangleIntake, turretServos, pedro);
    }



    @Override
    public void periodic(){


    }
}
