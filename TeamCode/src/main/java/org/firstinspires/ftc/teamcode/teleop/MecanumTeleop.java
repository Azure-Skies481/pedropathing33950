package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;


@TeleOp
@Configurable
public class MecanumTeleop extends LinearOpMode {

    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private Servo gate = null; //new servo js added
    double driveSpeed;
    private double maxspeed = 2800;
    @Sorter(sort = 0) double feedback = 0.001;

    boolean wasPressedLastFrame = false;

    boolean gateWasPressedLastFrame = false; //satoru...

    boolean gateOpen = false; //suguru...
    boolean shooterToggle = false;
    boolean intakeUsedLastFrame = false;

    @Sorter(sort = 1) int power = 1400;


    @Override
    public void runOpMode() throws InterruptedException {
        // Drivetrain motors (unchanged)
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontleftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backleftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontrightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backrightMotor");

        // Directions (match dualmotor style)
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);// start at CLOSE target

        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");

        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added



        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            driveSpeed = 0.4;

            driveSpeed += gamepad1.right_trigger*0.6;


            // --- Drive (unchanged) ---
            double y = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y);
            double x = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
            double rx = Math.pow(gamepad1.right_stick_x, 3.0);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = driveSpeed*((y + x + rx) / denominator);
            double backLeftPower = driveSpeed*((y - x + rx) / denominator);
            double frontRightPower = driveSpeed*((y - x - rx) / denominator);
            double backRightPower = driveSpeed*((y + x - rx) / denominator);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            double actualspeed = shooter.getVelocity();



            if (gamepad2.dpad_up){
                power = 1400;
            } else if (gamepad2.dpad_left){
                power = 1250;
            } else if (gamepad2.dpad_down){
                power = 1150;
            } else if (gamepad2.dpad_right) power = 1650;

            if (intakeUsedLastFrame && gamepad2.right_trigger==0 && gateOpen) gate.setPosition(0.0);

            if (gamepad2.right_bumper) {
                gateWasPressedLastFrame = true;
            } else {
                if (gateWasPressedLastFrame) {
                    gateOpen = !gateOpen;
                }
                gateWasPressedLastFrame = false;
            }

            if (gateOpen) {
                //open
                gate.setPosition(0.5);
            } else {
                //close
                gate.setPosition(0.0);
            }
            //gate.setPosition(0.5);
            if (intakeUsedLastFrame && gamepad2.right_trigger == 0) {
                gateOpen = false;
            }



            if (gamepad2.left_bumper) {
                wasPressedLastFrame = true;
            } else {
                if (wasPressedLastFrame) {
                    shooterToggle = !shooterToggle;
                }
                wasPressedLastFrame = false;
            }
            if (shooterToggle) {

                this.shooter.setPower((feedback * (power - actualspeed) + actualspeed/maxspeed));
            } else {
                this.shooter.setVelocity(0);
            }

            intake.setPower(gamepad2.right_trigger * -0.5 + gamepad2.left_trigger);
            intakeUsedLastFrame = gamepad2.right_trigger > 0;
            telemetry.addData("Shooter Real Velocity", shooter.getVelocity() * 28);
            telemetry.addData("Shooter Target Velocity", power);
            telemetry.addData("Gate Open?", gateOpen);
            telemetry.update();
        }
    }
}