package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.Angle;

public class Intake implements Mechanism {
    private double currentAngle;
    private double currentVel;
    public double offset;
    NextMotor intakeMotor = new NextMotor("intakeMotor");
    NextMotor intakeMotor2 = new NextMotor("intakeMotor2");
    private static final double DEG_PER_COUNT = 360.0 / 4000.0;

    public Intake(){
        intakeMotor.setAnglePerCount(new Angle(DEG_PER_COUNT, Units.Degrees));
    }
    public Command run() {
        return instant(() -> intakeMotor.setThrottle(-0.75)).requiring(intakeMotor);
    }
    public Command stop(){
        return instant(() -> intakeMotor.setThrottle(0.0)).requiring(intakeMotor);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }
    public double getActualAngle(){
        return currentAngle-offset;
    }
    public void resetEncoder(){
        offset = currentAngle;
    }

    public double getCurrentVel() {
        return currentVel;
    }
    @Override
    public void periodic() {
        intakeMotor2.setThrottle(intakeMotor.getThrottle());
        currentAngle = intakeMotor.getEncoderPosition().into(Units.Degrees);
        currentVel = intakeMotor.getEncoderVelocity().getMagnitude();

    }

}
