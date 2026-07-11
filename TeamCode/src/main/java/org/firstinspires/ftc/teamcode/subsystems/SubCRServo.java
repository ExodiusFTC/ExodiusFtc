package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;
import dev.nextftc.hardware.impl.FeedbackServoEx;

public class SubCRServo implements Subsystem {
    public static final SubCRServo INSTANCE = new SubCRServo();
    private SubCRServo(){}
    FeedbackCRServoEx servo = new FeedbackCRServoEx(
            0.01, // Or your preferred cache tolerance
            () ->  ActiveOpMode.hardwareMap().analogInput.get("analog-name"),
            () ->  ActiveOpMode.hardwareMap().crservo.get("servo-name")
    );
    double totalAngle = 0.0; // This is your angle of the servo
    double previousAngle = 0.0; // This is the previous loop's servo position

    void updatePosition() {
        double currentAngle = servo.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

        totalAngle += deltaAngle;
        previousAngle = currentAngle;
    }

    @Override
    public void initialize(){
        servo.setPower(0);
    }

    @Override
    public void periodic(){
        updatePosition();
    }

}
