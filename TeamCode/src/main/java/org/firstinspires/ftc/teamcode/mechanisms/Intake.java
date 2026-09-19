package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor intakeMotor = new NextMotor("intakeMotor");
    public Intake(){
        intakeMotor.getVelocityConstants().setKP(0.0);
        intakeMotor.getVelocityConstants().setKI(0.0);
        intakeMotor.getVelocityConstants().setKD(0.0);
        intakeMotor.getVelocityConstants().setKS(0.0);
        intakeMotor.getVelocityConstants().setKV(0.0);

    }
    public Command run() {
        return instant(() -> intakeMotor.setThrottle(0.8)).requiring(intakeMotor);
    }
    public Command stop(){
        return instant(() -> intakeMotor.setThrottle(0.0)).requiring(intakeMotor);
    }
    @Override
    public void periodic() {


    }

}
