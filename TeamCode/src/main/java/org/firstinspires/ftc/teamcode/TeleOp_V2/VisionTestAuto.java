package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Vision Test Auto")
public class VisionTestAuto extends LinearOpMode {

    private DcMotor FL, FR, BL, BR;
    private ArtifactVision vision;

    @Override
    public void runOpMode() {

        FL = hardwareMap.get(DcMotor.class, "frontLeft");
        FR = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");

        // reverse whichever side needs it for the robot
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        vision = new ArtifactVision(hardwareMap, 0);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            vision.update();

            if (vision.hasTarget()) {
                double tx = vision.getTx();
                double ta = vision.getTa();
                boolean aligned = vision.isAligned();

                if (aligned) {
                    stopStrafe();
                } else {
                    double power = vision.getStrafePower();
                    strafe(power);
                }

                telemetry.addData("Target", "YES");
                telemetry.addData("tx", tx);
                telemetry.addData("ta", ta);
                telemetry.addData("Aligned", aligned);
                telemetry.addData("Strafe Power", vision.getStrafePower());
            } else {
                stopStrafe();
                telemetry.addData("Target", "NO");
            }

            telemetry.update();
        }
    }

    // basic mecanum strafe — positive = right, negative = left (check this matches your bot)
    private void strafe(double power) {
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(-power);
        BR.setPower(power);
    }

    private void stopStrafe() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}