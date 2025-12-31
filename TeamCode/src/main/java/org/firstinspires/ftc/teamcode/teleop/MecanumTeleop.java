package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        // Shooter and intake
        DcMotor shooter = hardwareMap.dcMotor.get("shooterMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        // Directions (match dualmotor style)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Flywheel controller (PID/RPM)
        Flywheel flywheel = new Flywheel(shooter, telemetry);
        boolean shooterOn = true;          // start ON (matches original behavior)
        flywheel.setShooterOn(shooterOn);
        flywheel.setModeFar(false);        // start at CLOSE target

        boolean dpadLeftLast = false;
        boolean dpadRightLast = false;
        boolean dpadDownLast = false;      // for toggle edge-detect

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // --- Drive (unchanged) ---
            double y = -gamepad1.left_stick_y; // Y is reversed
            double x = gamepad1.left_stick_x;  // Strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // --- Intake control (gamepad1 triggers) ---
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger; // RT fwd, LT rev
            intakeMotor.setPower(intakePower);

            // --- Shooter RPM control (Flywheel) ---
            // D-pad right = FAR, D-pad left = CLOSE (edge-detected)
            boolean dpadRightNow = gamepad2.dpad_right;
            boolean dpadLeftNow = gamepad2.dpad_left;
            if (dpadRightNow && !dpadRightLast) {
                flywheel.setModeFar(true);
            }
            if (dpadLeftNow && !dpadLeftLast) {
                flywheel.setModeFar(false);
            }
            dpadRightLast = dpadRightNow;
            dpadLeftLast = dpadLeftNow;

            // --- Shooter on/off toggle (D-pad down) ---
            boolean dpadDownNow = gamepad2.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                shooterOn = !shooterOn;              // toggle state
                flywheel.setShooterOn(shooterOn);    // apply to flywheel
            }
            dpadDownLast = dpadDownNow;

            // Update flywheel PID; no calibration button in this OpMode
            flywheel.update(nowMs, false);

            // --- Telemetry ---
            telemetry.addData("Drive", "FL:%.2f BL:%.2f FR:%.2f BR:%.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("Shooter Target RPM", "%.1f", flywheel.getTargetRPM());
            telemetry.addData("Shooter Current RPM", "%.1f", flywheel.getCurrentRPM());
            telemetry.addData("Shooter Power", "%.3f", flywheel.getLastAppliedPower());
            telemetry.update();
        }
    }
}