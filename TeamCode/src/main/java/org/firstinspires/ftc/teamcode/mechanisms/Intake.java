package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Intake implements Mechanism {
    private double currentAngle;
    private double currentVel;
    NextMotor intakeMotor = new NextMotor("intakeMotor");
    NextMotor intakeMotor2 = new NextMotor("intakeMotor2");
    public Intake(){

    }
    public Command run() {
        return instant(() -> intakeMotor.setThrottle(-0.6)).requiring(intakeMotor);
    }
    public Command stop(){
        return instant(() -> intakeMotor.setThrottle(0.0)).requiring(intakeMotor);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getCurrentVel() {
        return currentVel;
    }
    @Override
    public void periodic() {
        intakeMotor2.setThrottle(intakeMotor.getThrottle());
        currentAngle = intakeMotor.getEncoderPosition().getMagnitude();
        currentVel = intakeMotor.getEncoderVelocity().getMagnitude();

    }

}
