package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelModified;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@TeleOp
@Configurable
public class HORSreplica extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private Servo gate = null;
    private VoltageSensor voltageSensor = null;

    private double driveSpeed;

    @Sorter(sort = 0)
    public static double feedback = 0.0008; // not used anymore

    private boolean shooterToggle = true;  // Start with shooter ON
    private boolean shooterToggleWasPressed = false;

    private boolean gateOpen = false;
    private boolean gateToggleWasPressed = false;

    private boolean fastMode = false;
    private boolean fastModeWasPressed = false;

    private boolean imuReferenceResetWasPressed = false;

    private boolean dpadLeftWasPressed = false;
    private boolean dpadRightWasPressed = false;

    private double imuAlignAngle;

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Sorter(sort = 1)
    public static int power = 3600;  // DEFAULT RPM IS 2600

    private IMU imu = null;
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
    ElapsedTime Timer = new ElapsedTime();

    // Shooter control object with dual motors
    private FlywheelModified flywheel;

    // Rumble state check
    private boolean wasVibratingLastLoop = false;

    // IMU Align
    public void getImuAlignAngle() {
        imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
    }

    public void imuAlign() {
        double timeout = 0.5;
        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();

        while (opModeIsActive()) {
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = imuAlignAngle - imuAngle;
            double alignPower = 0.02 * error;

            frontLeftMotor.setPower(-alignPower);
            backLeftMotor.setPower(-alignPower);
            frontRightMotor.setPower(alignPower);
            backRightMotor.setPower(alignPower);

            double velocity = (frontLeftMotor.getVelocity() + backLeftMotor.getVelocity()
                    + frontRightMotor.getVelocity() + backRightMotor.getVelocity()) / 4;

            telemetry.addData("imu: ", imuAngle);
            telemetry.addData("error: ", error);
            telemetry.update();

            if (Math.abs(error) <= 2.5 && Math.abs(velocity) <= 0.3) {
                telemetry.addData("skibidi", "yes it's done yo");
                telemetry.update();
                break;
            }
            if (alignTimer.seconds() > timeout) {
                telemetry.addData("imuAlign", "Timeout reached, aborting");
                telemetry.update();
                break;
            }
            idle();
        }
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Launch System
    public void launchSystem() {
        stopMove();
        Timer.reset();
        double skibidi = Timer.time(TimeUnit.MILLISECONDS);

        gate.setPosition(0.0);  // Open gate
        intake.setPower(-0.85);
        while (skibidi < 1000 && opModeIsActive()) {
            skibidi = Timer.time(TimeUnit.MILLISECONDS);
            flywheel.update(); // Keep updating during launch
        }
        intake.setPower(0);
        gate.setPosition(0.3);  // Close gate
    }

    public void stopMove() {
        frontLeftMotor.setVelocity(0);
        backLeftMotor.setVelocity(0);
        frontRightMotor.setVelocity(0);
        backRightMotor.setVelocity(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("backRight");

        // Drivetrain directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Configure shooter direction - FORWARD for positive power = correct spin
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize second shooter motor
        try {
            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            // shooter2 spins OPPOSITE direction mechanically, so REVERSE makes it spin same effective direction
            shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
            telemetry.addData("Shooter2", "Initialized ✓");
        } catch (IllegalArgumentException e) {
            shooter2 = null;
            telemetry.addData("Shooter2", "Not found (running single motor)");
        }

        gate = hardwareMap.get(Servo.class, "gateServo");

        // Get voltage sensor
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize FlywheelModified with both motors
        flywheel = new FlywheelModified(shooter, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRPM(power);
        flywheel.setShooterOn(false);  // Keep shooter OFF during init

        // Set gate to closed position during init
        gate.setPosition(0.3);

        imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Dual Motors", shooter2 != null ? "YES" : "NO");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Turn shooter ON at START (not during init)
        shooterToggle = true;
        flywheel.setShooterOn(true);

        double time;
        while (opModeIsActive()) {
            time = Timer.time(TimeUnit.MILLISECONDS);

            // Auto Launch System Check
            if (gamepad1.y || gamepad2.y) {
                launchSystem();
            }

            // Fast mode toggle (gamepad1 right bumper rising edge)
            if (gamepad1.right_bumper) {
                if (!fastModeWasPressed) {
                    fastMode = !fastMode;
                }
                fastModeWasPressed = true;
            } else {
                fastModeWasPressed = false;
            }

            driveSpeed = fastMode ? 1.0 : 0.4;
            driveSpeed += gamepad1.right_trigger * (fastMode ? 0.0 : 0.6);

            // --- Drive ---
            double y = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
            double rx = Math.pow(gamepad1.right_stick_x, 3.0);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = driveSpeed * ((y + x + rx) / denominator);
            double backLeftPower = driveSpeed * ((y - x + rx) / denominator);
            double frontRightPower = driveSpeed * ((y - x - rx) / denominator);
            double backRightPower = driveSpeed * ((y + x - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Changing shooter power
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                if (!dpadLeftWasPressed) {
                    power = Math.max(0, power - 50);
                    flywheel.setTargetRPM(power);
                }
                dpadLeftWasPressed = true;
            } else {
                dpadLeftWasPressed = false;
            }

            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (!dpadRightWasPressed) {
                    power = power + 50;
                    flywheel.setTargetRPM(power);
                }
                dpadRightWasPressed = true;
            } else {
                dpadRightWasPressed = false;
            }

            // Intake control
            double intakePower = 0.0;
            if (gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) {
                intakePower = gamepad1.left_trigger * 0.5 - gamepad1.right_trigger;
            } else {
                intakePower = gamepad2.left_trigger * 0.5 - gamepad2.right_trigger * 1;
            }
            intake.setPower(intakePower);

            // Toggle shooter on/off
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                if (!shooterToggleWasPressed) {
                    shooterToggle = !shooterToggle;
                    flywheel.setShooterOn(shooterToggle);
                }
                shooterToggleWasPressed = true;
            } else {
                shooterToggleWasPressed = false;
            }

            // Gate toggle
            if (gamepad1.b || gamepad2.b) {
                if (!gateToggleWasPressed) {
                    gateOpen = !gateOpen;
                }
                gateToggleWasPressed = true;
            } else {
                gateToggleWasPressed = false;
            }

            // Reset IMU reference point
            if (gamepad1.a || gamepad2.a) {
                if (!imuReferenceResetWasPressed) {
                    imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
                }
                imuReferenceResetWasPressed = true;
            } else {
                imuReferenceResetWasPressed = false;
            }

            // Gate position logic
            gate.setPosition(gateOpen ? 0.0 : 0.3);

            // Update PIDF controller
            flywheel.update();

            // Rumble feedback when not at speed
            boolean atTarget = flywheel.isAtTarget();

            if (flywheel.isShooterOn() && !atTarget) {
                gamepad1.rumble(0.5, 0.5, 100);
                gamepad2.rumble(0.5, 0.5, 100);
                wasVibratingLastLoop = true;
            } else if (wasVibratingLastLoop) {
                gamepad1.stopRumble();
                gamepad2.stopRumble();
                wasVibratingLastLoop = false;
            }

            // IMU Align
            if (gamepad1.x) {
                imuAlign();
            }

            // Telemetry
            telemetry.addData("Shooter RPM", "%.0f", flywheel.getCurrentRPM());
            telemetry.addData("Target RPM", "%.0f", flywheel.getTargetRPM());
            telemetry.addData("Power", "%.2f", flywheel.getLastAppliedPower());
            telemetry.addData("At Speed", flywheel.isAtTarget() ? "✓" : "✗");
            telemetry.addData("Gate Open", gateOpen);
            telemetry.addData("Shooter On", flywheel.isShooterOn());
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Fast Mode", fastMode);
            telemetry.addData("Dual Motors", flywheel.isDualMotor() ? "YES" : "NO");
            telemetry.addData("Reference Heading", "%.1f°", Math.toDegrees(imuAlignAngle));
            telemetry.addData("Current Heading", "%.1f",(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.update();
        }
    }
}