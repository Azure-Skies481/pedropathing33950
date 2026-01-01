package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleop extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;


    boolean wasPressedLastFrame = false;
    boolean shooterToggle = false;

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

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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



            if (gamepad1.dpad_up){
                power = power + 0.5;
            } else if (gamepad1.dpad_down){
                power = power - 0.5;
            }


            if (gamepad1.left_bumper) {
                wasPressedLastFrame = true;
            } else {
                if (wasPressedLastFrame) {
                    shooterToggle = !shooterToggle;

                    if (shooterToggle) {
                        this.shooter.setVelocity(power);
                    } else {
                        this.shooter.setVelocity(0);
                    }
                }
                wasPressedLastFrame = false;
            }

            intake.setVelocity(gamepad1.right_trigger * 1000);
            telemetry.addData("Shooter Real Velocity", shooter.getVelocity());
            telemetry.addData("Shooter Target Velocity", power);
            telemetry.update();
        }
    }
}