package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleop extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private Servo servoGate = null;

    boolean wasPressedLastFrame = false;
    boolean shooterToggle = false;

    boolean gateWasPressedLastFrame = false;
    boolean gateToggle = false; // false = closed (0.9), true = open (0.1)

    double power = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        // Directions (match dualmotor style)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);// start at CLOSE target

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        servoGate = hardwareMap.get(Servo.class, "servogate");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize gate closed
        servoGate.setPosition(0.9);

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

            // Speed adjust: right = increase, left = decrease
            boolean powerChanged = false;
            if (gamepad1.dpad_right){
                power = power + 0.5;
                powerChanged = true;
            } else if (gamepad1.dpad_left){
                power = power - 0.5;
                powerChanged = true;
            }
            // If shooter is on, update to new power immediately
            if (powerChanged && shooterToggle) {
                shooter.setVelocity(power);
            }

            // Shooter toggle on dpad_down
            if (gamepad1.dpad_down) {
                wasPressedLastFrame = true;
            } else {
                if (wasPressedLastFrame) {
                    shooterToggle = !shooterToggle;
                    if (shooterToggle) {
                        shooter.setVelocity(power);
                    } else {
                        shooter.setVelocity(0);
                    }
                }
                wasPressedLastFrame = false;
            }

            // Intake: right trigger forward, left trigger reverse
            double intakeCommand = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setVelocity(intakeCommand * 1000);

            // Gate toggle on Y: open 0.1, closed 0.9
            if (gamepad1.y) {
                gateWasPressedLastFrame = true;
            } else {
                if (gateWasPressedLastFrame) {
                    gateToggle = !gateToggle;
                    servoGate.setPosition(gateToggle ? 0.1 : 0.9);
                }
                gateWasPressedLastFrame = false;
            }

            telemetry.addData("Shooter Real Velocity", shooter.getVelocity());
            telemetry.addData("Shooter Target Velocity", power);
            telemetry.addData("Intake Command", intakeCommand);
            telemetry.addData("Gate Position", servoGate.getPosition());
            telemetry.update();
        }
    }
}