package org.firstinspires.ftc.teamcode;



import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.TriangleIntake;
import org.firstinspires.ftc.teamcode.mechanisms.TurretServos;

import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.drive.DriveCommands;

public class Robot implements NextRobot {

//    PedroMechanism pedro = new PedroMechanism(Constants::create);

    private final TurretServos turretServos = new TurretServos();
    private final TriangleIntake triangleIntake = new TriangleIntake();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    //private final ELCencoder encoder = new ELCencoder();
    private final Drivetrain drivetrain = new Drivetrain();
    //private final TurretEncoder turretEncoder = new TurretEncoder();


    public Robot() {}
//    public void initHardware(HardwareMap hwMap) {
//        encoder.init(hwMap);
//    }
//    public ELCencoder getEncoder() {
//        return encoder;
//    }
//    public TurretEncoder getTurret(){
//        return turretEncoder;
//    }
    public TurretServos getTurretServos(){
        return turretServos;
    }
    public Intake getIntake(){
        return intake;
    }
    public Shooter getShooter(){
        return shooter;
    }
    public TriangleIntake getTriangleIntake(){
        return triangleIntake;
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

    public void startDrive(Gamepad gamepad1) {
        DriveCommands.mecanumDrive(
                drivetrain.frontLeft,
                drivetrain.frontRight,
                drivetrain.backLeft,
                drivetrain.backRight,
                gamepad1
        ).schedule();
    }


    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, shooter, drivetrain, triangleIntake, turretServos);
    }



    @Override
    public void periodic(){


    }
}
