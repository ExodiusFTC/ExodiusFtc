package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Intake Linear TeleOp", group = "TeleOp")
public class BasicIntakeTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intakeMotor.setPower(0.7);


            } else if (gamepad1.b) {
                intakeMotor.setPower(0.0);
            }

            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.update();
        }
    }
}