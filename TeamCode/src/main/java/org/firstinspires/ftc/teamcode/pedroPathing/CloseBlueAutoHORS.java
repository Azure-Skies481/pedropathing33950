package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Close Blue Auto 🔷", group = "auto")
@Configurable
public class CloseBlueAutoHORS extends LinearOpMode {

    // Tunables (can be adjusted in dashboard)
    public static double TARGET_SHOOTER_RPM = 950;
    public static double RPM_TOLERANCE = 75;
    public static double SHOOTER_KP = 0.003;     // same style as TeleOp feedback
    public static double SHOOTER_MAX_RPM = 2570; // for feedforward term
    public static double DRIVE_POWER = 0.5;
    public static double STRAFE_POWER = 0.5;
    public static int DRIVE_BACK_TICKS = 1500;   // distance back (positive number; code drives negative)
    public static int STRAFE_TICKS = 200;       // distance to strafe (positive = right, negative = left)
    public static long SPINUP_TIMEOUT_MS = 3000;
    public static double INTAKE_POWER = 1.0;

    // Hardware
    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotorEx shooter;
    private DcMotorEx intake;
    private Servo gate;

    // Gate positions
    private static final double GATE_OPEN = 0.1;
    private static final double GATE_CLOSED = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        // dt
        frontLeft  = hardwareMap.dcMotor.get("frontleftMotor");
        backLeft   = hardwareMap.dcMotor.get("backleftMotor");
        frontRight = hardwareMap.dcMotor.get("frontrightMotor");
        backRight  = hardwareMap.dcMotor.get("backrightMotor");


        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter & intake
        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Gate
        gate = hardwareMap.get(Servo.class, "servogate");
        setGate(false); // closed

        // Prepare drive encoders
        resetDriveEncoders();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Auto Back-Strafe Shoot ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // 1) Spin shooter up and wait until close to target (or timeout)
        spinShooterToTarget();
        waitForShooterAtSpeed(SPINUP_TIMEOUT_MS);

        // 2) Turn on intake and keep it on
        intake.setPower(INTAKE_POWER);

        // 3) Drive straight back using encoders
        driveStraightTicks(-Math.abs(DRIVE_BACK_TICKS), DRIVE_POWER);

        // 4) Open gate
        setGate(true);

        // 5) Strafe for configurable distance
        strafeTicks(STRAFE_TICKS, STRAFE_POWER);

        // 6) Stop everything
        stopAll();
    }

    private void spinShooterToTarget() {
        // Simple P + feedforward similar to TeleOp
        double actual = -shooter.getVelocity();
        double power = SHOOTER_KP * (TARGET_SHOOTER_RPM - actual) + (actual / SHOOTER_MAX_RPM);
        shooter.setPower(power);
    }

    private void waitForShooterAtSpeed(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start < timeoutMs)) {
            double actual = -shooter.getVelocity();
            double error = TARGET_SHOOTER_RPM - actual;
            double power = SHOOTER_KP * error + (actual / SHOOTER_MAX_RPM);
            shooter.setPower(power);

            telemetry.addData("Target RPM", TARGET_SHOOTER_RPM);
            telemetry.addData("Actual RPM", actual);
            telemetry.addData("Error", error);
            telemetry.update();

            if (Math.abs(error) <= RPM_TOLERANCE) break;
        }
    }

    private void driveStraightTicks(int ticks, double power) {
        resetDriveEncoders();
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy())) {
            telemetry.addData("Driving back ticks", ticks);
            telemetry.update();
        }

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void strafeTicks(int ticks, double power) {
        resetDriveEncoders();
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Positive ticks = strafe right, negative = strafe left
        frontLeft.setTargetPosition(ticks);
        backLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || backLeft.isBusy() || frontRight.isBusy() || backRight.isBusy())) {
            telemetry.addData("Strafing ticks", ticks);
            telemetry.update();
        }

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAll() {
        stopDrive();
        shooter.setPower(0);
        intake.setPower(0);
        setGate(false); // close
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private void setGate(boolean open) {
        if (gate != null) {
            gate.setPosition(open ? GATE_OPEN : GATE_CLOSED);
        }
    }

    private void resetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        backLeft.setMode(mode);
        frontRight.setMode(mode);
        backRight.setMode(mode);
    }
}