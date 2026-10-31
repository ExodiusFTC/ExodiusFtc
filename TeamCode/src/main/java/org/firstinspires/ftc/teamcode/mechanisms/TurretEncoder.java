package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.Angle;

public class TurretEncoder implements Mechanism {
    // 4000 cpr
    // 45:160 teeth
    // 1:3.55555555556
    // (0.09 x (45/160)) = angle per count
    NextMotor shaftEncoder = new NextMotor("Encoder");
    NextCRServo turretServo1 = new NextCRServo("turretServo1");
    NextCRServo turretServo2 = new NextCRServo("turretServo2");

    double anglePerCount = (0.09 * (45.0 / 160.0));
    private double currentAngle;
    private double currentVel;
    private double offsetDeg = 0;



    public TurretEncoder() {
        shaftEncoder.setAnglePerCount(new Angle(anglePerCount, Units.Degrees));

    }
    public Command setPower(double power){
        return instant(() -> turretServo1.setPower(-1)).requiring(turretServo1);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getCurrentVel() {
        return currentVel;
    }

    public void resetEncoder() {
        offsetDeg = shaftEncoder.getEncoderPosition().into(Units.Degrees);
        currentAngle = 0;
    }

    @Override
    public void periodic() {
        currentAngle = shaftEncoder.getEncoderPosition().into(Units.Degrees) - offsetDeg;
        currentVel = shaftEncoder.getEncoderVelocity().into(Units.DegreesPerSecond);
        turretServo2.setPower(turretServo1.getPower());
    }
}