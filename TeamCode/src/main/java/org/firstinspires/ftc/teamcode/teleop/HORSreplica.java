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

@TeleOp
@Configurable
public class HORSreplica extends LinearOpMode {

    private DcMotorEx intake;
    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private Servo gate;
    private Servo hoodservo;
    private VoltageSensor voltageSensor;

    private double driveSpeed;

    // Toggle edge-detection flags
    private boolean shooterToggle = true;
    private boolean shooterToggleWasPressed = false;
    private boolean gateOpen = false;
    private boolean fastMode = true;
    private boolean fastModeWasPressed = false;
    private boolean imuReferenceResetWasPressed = false;
    private boolean dpadLeftWasPressed = false;
    private boolean dpadRightWasPressed = false;

    private double imuAlignAngle;

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Sorter(sort = 1)
    public static int RPM_current = 3700;

    private IMU imu;
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));

    private FlywheelModified flywheel;

    // ========== NON-BLOCKING STATE MACHINES ==========

    // Launch system state machine (replaces blocking 1-second wait)
    private enum LaunchState { IDLE, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;
    private final ElapsedTime launchTimer = new ElapsedTime();

    // IMU align state machine (replaces blocking while-loop)
    private enum AlignState { IDLE, ALIGNING }
    private AlignState alignState = AlignState.IDLE;
    private final ElapsedTime alignTimer = new ElapsedTime();
    private static final double ALIGN_TIMEOUT = 0.5;
    private static final double ALIGN_P = 0.02;
    private static final double ALIGN_TOLERANCE_DEG = 2.5;
    private static final double ALIGN_VELOCITY_TOLERANCE = 0.3;

    // Telemetry throttle — only update every N ms to cut USB overhead
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_INTERVAL_MS = 100; // 10 Hz is plenty for humans

    // Cached servo positions to avoid redundant USB writes
    private double lastGatePosition = -1;
    private double lastHoodPosition = -1;

    // ========== HELPER: Write servo only if position changed ==========
    private void setGatePosition(double pos) {
        if (Math.abs(pos - lastGatePosition) > 0.005) {
            gate.setPosition(pos);
            lastGatePosition = pos;
        }
    }

    private void setHoodPosition(double pos) {
        if (Math.abs(pos - lastHoodPosition) > 0.005) {
            hoodservo.setPosition(pos);
            lastHoodPosition = pos;
        }
    }

    // ========== NON-BLOCKING IMU ALIGN (called each loop tick) ==========
    private void startImuAlign() {
        if (alignState == AlignState.IDLE) {
            alignState = AlignState.ALIGNING;
            alignTimer.reset();
        }
    }

    private void updateImuAlign() {
        if (alignState != AlignState.ALIGNING) return;

        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = imuAlignAngle - imuAngle;
        double alignPower = ALIGN_P * error;

        frontLeftMotor.setPower(-alignPower);
        backLeftMotor.setPower(-alignPower);
        frontRightMotor.setPower(alignPower);
        backRightMotor.setPower(alignPower);

        double velocity = (frontLeftMotor.getVelocity() + backLeftMotor.getVelocity()
                + frontRightMotor.getVelocity() + backRightMotor.getVelocity()) / 4.0;

        boolean settled = Math.abs(error) <= ALIGN_TOLERANCE_DEG && Math.abs(velocity) <= ALIGN_VELOCITY_TOLERANCE;
        boolean timedOut = alignTimer.seconds() > ALIGN_TIMEOUT;

        if (settled || timedOut) {
            // Stop motors and return to normal drive
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            alignState = AlignState.IDLE;
        }
    }

    // ========== NON-BLOCKING LAUNCH SYSTEM (called each loop tick) ==========
    private void startLaunch() {
        if (launchState == LaunchState.IDLE) {
            launchState = LaunchState.LAUNCHING;
            launchTimer.reset();
            // Open gate + run intake
            setGatePosition(0.3);
            intake.setPower(-1);
            // Stop drive motors during launch
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }

    private void updateLaunch() {
        if (launchState != LaunchState.LAUNCHING) return;

        if (launchTimer.milliseconds() >= 1000) {
            intake.setPower(0);
            setGatePosition(0.1);
            launchState = LaunchState.IDLE;
        }
        // flywheel.update() is already called once per loop below — no need to double-call
    }

    // ========== MAIN OPMODE ==========
    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- Hardware Init ----------
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor  = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRight");

        // Set zero-power behavior ONCE at init, not repeatedly
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        try {
            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
            telemetry.addData("Shooter2", "Initialized ✓");
        } catch (IllegalArgumentException e) {
            shooter2 = null;
            telemetry.addData("Shooter2", "Not found (running single motor)");
        }

        gate = hardwareMap.get(Servo.class, "gateServo");
        hoodservo = hardwareMap.get(Servo.class, "hoodservo");

        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        flywheel = new FlywheelModified(shooter, shooter2, telemetry, voltageSensor);
        flywheel.setTargetRPM(RPM_current);
        flywheel.setShooterOn(false);

        setGatePosition(0.3);

        imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Dual Motors", shooter2 != null ? "YES" : "NO");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Turn shooter ON at start
        shooterToggle = true;
        flywheel.setShooterOn(true);
        telemetryTimer.reset();

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {

            boolean isLaunching = (launchState == LaunchState.LAUNCHING);
            boolean isAligning  = (alignState == AlignState.ALIGNING);

            // --- Non-blocking launch system ---
            if ((gamepad1.y || gamepad2.y) && !isLaunching && !isAligning) {
                startLaunch();
            }
            updateLaunch();

            // --- Non-blocking IMU align ---
            if (gamepad1.x && !isAligning && !isLaunching) {
                startImuAlign();
            }
            updateImuAlign();

            // --- Drive (skip if launching or aligning — those control motors) ---
            if (!isLaunching && !isAligning) {

                // Fast mode toggle
                if (gamepad1.right_bumper) {
                    if (!fastModeWasPressed) fastMode = !fastMode;
                    fastModeWasPressed = true;
                } else {
                    fastModeWasPressed = false;
                }

                driveSpeed = fastMode ? 1.0 : 0.4;

                // Squared inputs for fine control at low stick values
                double y  = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
                double x  =  gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
                double rx =  gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                frontLeftMotor.setPower(driveSpeed * (y + x + rx) / denom);
                backLeftMotor.setPower(driveSpeed * (y - x + rx) / denom);
                frontRightMotor.setPower(driveSpeed * (y - x - rx) / denom);
                backRightMotor.setPower(driveSpeed * (y + x - rx) / denom);
            }

            // --- RPM adjustment (dpad left/right) ---
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                if (!dpadLeftWasPressed) {
                    RPM_current = Math.max(0, RPM_current - 50);
                    flywheel.setTargetRPM(RPM_current);
                }
                dpadLeftWasPressed = true;
            } else {
                dpadLeftWasPressed = false;
            }

            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (!dpadRightWasPressed) {
                    RPM_current += 50;
                    flywheel.setTargetRPM(RPM_current);
                }
                dpadRightWasPressed = true;
            } else {
                dpadRightWasPressed = false;
            }

            // --- Intake (no change to logic, just cleaner) ---
            double intakePower;
            if (gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) {
                intakePower = gamepad1.left_trigger * 0.5 - gamepad1.right_trigger;
            } else {
                intakePower = gamepad2.left_trigger * 0.5 - gamepad2.right_trigger;
            }
            // Only set during launch if launch isn't controlling intake
            if (!isLaunching) {
                intake.setPower(intakePower);
            }

            // --- Shooter toggle ---
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                if (!shooterToggleWasPressed) {
                    shooterToggle = !shooterToggle;
                    flywheel.setShooterOn(shooterToggle);
                }
                shooterToggleWasPressed = true;
            } else {
                shooterToggleWasPressed = false;
            }

            // --- IMU reference reset ---
            if (gamepad1.a || gamepad2.a) {
                if (!imuReferenceResetWasPressed) {
                    imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
                }
                imuReferenceResetWasPressed = true;
            } else {
                imuReferenceResetWasPressed = false;
            }

            // --- Hood presets (only write servo if position actually changes) ---
            if (gamepad2.dpad_up) {
                setHoodPosition(0.28);
                flywheel.setTargetRPM(4450);
            }
            if (gamepad2.right_bumper) {
                setHoodPosition(0.5);
                flywheel.setTargetRPM(3800);
            }
            if (gamepad2.left_bumper) {
                setHoodPosition(0.5);
                flywheel.setTargetRPM(3200);
            }

            // --- Flywheel PIDF update (exactly ONCE per loop) ---
            flywheel.update();

            // --- Throttled telemetry (10 Hz instead of ~50+ Hz) ---
            if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
                telemetryTimer.reset();
                telemetry.addData("Shooter RPM", "%.0f", flywheel.getCurrentRPM());
                telemetry.addData("Target RPM", "%.0f", flywheel.getTargetRPM());
                telemetry.addData("Power", "%.2f", flywheel.getLastAppliedPower());
                telemetry.addData("At Speed", flywheel.isAtTarget() ? "✓" : "✗");
                telemetry.addData("Shooter On", flywheel.isShooterOn());
                telemetry.addData("Fast Mode", fastMode);
                telemetry.addData("Launch", launchState);
                telemetry.addData("Align", alignState);
                telemetry.update();
            }
        }
    }
}