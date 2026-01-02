package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Configurable
public class IMUalign extends LinearOpMode {

    private DcMotorEx intake = null;
    private Servo servoGate = null;
    private IMU imu = null;

    // Runtime constants
    private static final double SHOOTER_MIN_RPM = 0.0;
    private static final long DPAD_DEBOUNCE_MS = 200;
    private static final double BOOST_ERROR_RPM = 200.0;
    private static final double BOOST_MULT = 1.15;

    // Servo gate positions
    private static final double GATE_OPEN = 0.1;
    private static final double GATE_CLOSED = 0.8;
    private boolean shooterEnabled = false;

    // Edge/debounce
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private long lastDpadLeftMs = 0;
    private long lastDpadRightMs = 0;

    // Gate toggle
    private boolean gateOpen = false;
    private boolean prevY = false;

    // Rumble state
    private boolean wasAtTarget = false;
    private boolean rumbleActive = false;

    // Heading hold
    private boolean aligningToStart = false;
    private boolean prevX = false;
    private double startHeadingDeg = 0.0;

    // Turn-to-heading gains
    private static final double ALIGN_KP = 0.00004;      // proportional gain
    private static final double ALIGN_MIN_PWR = 0.08;  // minimum turn power to overcome static friction
    private static final double ALIGN_MAX_PWR = 0.45;  // clamp turn power
    private static final double ALIGN_TOL_DEG = 10.5;   // stop when within tolerance (degrees)

    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        double maxSpeed = 2570;
        double feedback = 0.003;
        double targetShooterRPM = 650;

        // Directions (match dualmotor style)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter (flywheel) motor
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shootermotor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo gate
        servoGate = hardwareMap.get(Servo.class, "servogate");
        setGate(gateOpen);
        shooterEnabled = false;

        // IMU init (Expansion Hub IMU, logo left, USB up)
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        waitForStart();
        if (isStopRequested()) return;

        // Capture starting heading and enable shooter
        startHeadingDeg = getHeadingDeg();
        aligningToStart = false;
        shooterEnabled = true;

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();

            // --- Drive (with align override on rotation) ---
            double y = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
            double rxDriver = Math.pow(gamepad1.right_stick_x, 3.0);

            // Heading align trigger
            boolean xPressed = gamepad1.x;
            if (xPressed && !prevX) {
                aligningToStart = true;
            }
            prevX = xPressed;

            double rx;
            if (aligningToStart) {
                double heading = getHeadingDeg();
                double error = angleWrapDeg(startHeadingDeg - heading);
                if (Math.abs(error) <= ALIGN_TOL_DEG) {
                    aligningToStart = false;
                    rx = 0.0;
                } else {
                    double turnCmd = ALIGN_KP * error;
                    turnCmd = Range.clip(turnCmd, -ALIGN_MAX_PWR, ALIGN_MAX_PWR);
                    if (Math.abs(turnCmd) < ALIGN_MIN_PWR) {
                        turnCmd = Math.copySign(ALIGN_MIN_PWR, turnCmd);
                    }
                    rx = turnCmd;
                }
            } else {
                rx = rxDriver;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Intake control
            intake.setVelocity(gamepad1.right_trigger * 1000);

            // Shooter controls
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            // Toggle shooter on/off with dpad down (edge-triggered)
            if (dpadDown && !prevDpadDown) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled) {
                    shooterMotor.setPower(0);
                    stopRumble();
                } else {
                    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            if (dpadLeft && !prevDpadLeft) {
                targetShooterRPM -= 50;
            }

            if (dpadRight && !prevDpadRight) {
                targetShooterRPM += 50;
            }

            double actual = -shooterMotor.getVelocity();
            shooterMotor.setPower(feedback * (targetShooterRPM - actual) + (actual / maxSpeed));

            // Gate toggle on Y
            boolean yPressed = gamepad1.y;
            if (yPressed && !prevY) {
                gateOpen = !gateOpen;
                setGate(gateOpen);
            }
            prevY = yPressed;

            // Save button states
            prevDpadDown = dpadDown;
            prevDpadLeft = dpadLeft;
            prevDpadRight = dpadRight;

            // Telemetry
            double currentRpmDisplay = shooterMotor.getVelocity();
            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Target RPM", targetShooterRPM);
            telemetry.addData("Current RPM", -Math.round(currentRpmDisplay * 10.0) / 10.0);
            telemetry.addData("Heading (deg)", getHeadingDeg());
            telemetry.addData("Start Heading (deg)", startHeadingDeg);
            telemetry.addData("Aligning to Start?", aligningToStart);
            telemetry.update();
        }
    }

    private double getHeadingDeg() {
        // Yaw is the robot heading; IMU returns radians or degrees as requested
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double angleWrapDeg(double angle) {
        // Normalize to (-180, 180]
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private void startContinuousRumble() {
        // Use a long rumble duration (10 seconds) - will be stopped when leaving tolerance
        // This avoids the issue of short rumbles getting lost
        gamepad1.rumble(1.0, 1.0, 10000);
    }

    private void stopRumble() {
        // Stop any ongoing rumble
        gamepad1.stopRumble();
    }

    private void setGate(boolean open) {
        if (servoGate != null) {
            servoGate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        }
    }
}