package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        // Shooter and intake
        DcMotor shooter = hardwareMap.dcMotor.get("shooterMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        // Directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset and enable encoder on shooter to ensure ticks update
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Flywheel controller (same logic as main HORS shooter)
        Flywheel flywheel = new Flywheel(shooter, telemetry);
        flywheel.setShooterOn(true);
        flywheel.setModeFar(false);                       // start CLOSE
        flywheel.setTargetRPM(Flywheel.TARGET_RPM_CLOSE); // start 90 RPM

        boolean dpadLeftLast = false;
        boolean dpadRightLast = false;
        boolean dpadDownLast = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // --- Drive ---
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

            // --- Intake (gamepad1 triggers) ---
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            intakeMotor.setPower(intakePower);

            // --- Shooter RPM control (gamepad1 only) ---
            boolean dpadRightNow = gamepad1.dpad_right;
            boolean dpadLeftNow = gamepad1.dpad_left;
            if (dpadRightNow && !dpadRightLast) {
                flywheel.adjustTargetRPM(10.0);   // +10 and hold
            }
            if (dpadLeftNow && !dpadLeftLast) {
                flywheel.adjustTargetRPM(-10.0);  // -10 and hold
            }
            dpadRightLast = dpadRightNow;
            dpadLeftLast = dpadLeftNow;

            // Shooter on/off toggle
            boolean dpadDownNow = gamepad1.dpad_down;
            if (dpadDownNow && !dpadDownLast) {
                flywheel.toggleShooterOn();
            }
            dpadDownLast = dpadDownNow;

            // Update flywheel (no calibration button in this OpMode)
            flywheel.update(nowMs, false);

            // Rumble when at target
            if (flywheel.isAtTarget()) {
                try { gamepad1.rumble(200); } catch (Throwable ignored) {}
            }

            // --- Telemetry ---
            telemetry.addData("Drive", "FL:%.2f BL:%.2f FR:%.2f BR:%.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Shooter On", flywheel.isShooterOn());
            telemetry.addData("Shooter Target RPM", "%.1f", flywheel.getTargetRPM());
            telemetry.addData("Shooter Current RPM", "%.1f", flywheel.getCurrentRPM());
            telemetry.addData("Shooter Power", "%.3f", flywheel.getLastAppliedPower());
            telemetry.addData("Shooter Enc Pos", shooter.getCurrentPosition());
            telemetry.addData("Shooter Enc dTicks", flywheel.getLastDeltaTicks());
            telemetry.addData("Shooter Enc dT(ms)", flywheel.getLastDeltaTimeMs());
            telemetry.addData("Shooter Mode", shooter.getMode());
            telemetry.update();
        }
    }
}