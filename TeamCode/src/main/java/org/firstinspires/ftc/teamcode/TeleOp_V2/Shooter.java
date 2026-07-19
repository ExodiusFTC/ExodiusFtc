package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp(name = "Shooter Tune")
public class Shooter extends LinearOpMode {

    // shows up live in FTC Dashboard -> Configuration
    public static boolean pidtrue = false;
    public static PIDCoefficients pid = new PIDCoefficients(0.00, 0.00, 0.00);
    public static BasicFeedforwardParameters ff = new BasicFeedforwardParameters(0.00, 0.0, 0.00);
    public static double TARGET_VELOCITY = 1800.0;

    @Override
    public void runOpMode() {
        MotorEx motor = new MotorEx("SH");
        MotorEx motor2 = new MotorEx("SH2");
        MotorGroup shooters = new MotorGroup(motor, motor2);

        ControlSystem controlSystem = ControlSystem.builder()
                .velPid(pid)
                .basicFF(ff)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            controlSystem.setGoal(new KineticState(0.0, TARGET_VELOCITY));

            double power = controlSystem.calculate(
                    new KineticState(shooters.getCurrentPosition(), shooters.getVelocity())
            );

            if (pidtrue) {
                shooters.setPower(power);
            } else {
                shooters.setPower(0.0);
            }

            telemetry.addData("target velocity", TARGET_VELOCITY);
            telemetry.addData("actual velocity", shooters.getVelocity());
            telemetry.addData("power", power);
            telemetry.update();
        }
    }
}
