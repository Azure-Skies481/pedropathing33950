package org.firstinspires.ftc.teamcode.autonomous;



import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.ShootingHelp;

import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoFarRed extends LinearOpMode {
    ShootingHelp shootingHelp = new ShootingHelp();
    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private Servo gate = null; //new servo js added
    private double maxspeed = 2800;
    private double feedback = 0.001;
    private boolean gateOpen = false; //Suguru...
    private boolean shooterToggle = false;
    private double power = 1550;
    private double drivespeed;

    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private IMU imu = null;
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
    public void moveForward(double amount) {
        resetEncoders();
        while (opModeIsActive()) {
            double position = (double) (frontLeftMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition() +
                    frontRightMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;
            double error = amount - position;
            double power = 0.01 * error;
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(power);
            double velocity = (frontLeftMotor.getVelocity() + backLeftMotor.getVelocity() + frontRightMotor.getVelocity() + backRightMotor.getVelocity())/4;
            telemetry.addData("error", error);
            telemetry.addData("velocity", velocity);
            telemetry.addData("position", position);
            telemetry.update();
            if (Math.abs(error) <= 15 && velocity <= 0.3) break;
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void moveForwardTime(double time){
        ElapsedTime moveTimer = new ElapsedTime();
        while (opModeIsActive()){
            double currentTime = moveTimer.time(TimeUnit.MILLISECONDS);
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            if(Math.abs(time-currentTime)<=50) break;
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void strafe(double amount, boolean left) {
        while (opModeIsActive()){
            double position = (double) (frontLeftMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition() +
                    frontRightMotor.getCurrentPosition() + backRightMotor.getCurrentPosition()) / 4;
            double error = amount - position;
            double power = 0.01 * error;
            int skibidi = -1;
            if (left) skibidi = skibidi * -1;
            frontLeftMotor.setPower(power * skibidi);
            frontRightMotor.setPower(power * -skibidi);
            backLeftMotor.setPower(power * -skibidi);
            backRightMotor.setPower(power * skibidi);
        }
    }


    public void turn (double angle){
        imu.initialize(parameters);
        imu.resetYaw();
        resetEncoders();
        while (opModeIsActive()){
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = angle - imuAngle;
            double power = 0.02*error;
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            double velocity = (frontLeftMotor.getVelocity() + backLeftMotor.getVelocity() + frontRightMotor.getVelocity() + backRightMotor.getVelocity())/4;

            telemetry.addData("imu: ", imuAngle);
            telemetry.addData("error: ", error);
            telemetry.update();
            if (Math.abs(error) <= 2.5 && velocity<=0.3){
                telemetry.addData("skibidi", "yes it's done yo");
                telemetry.update();
                break;
            }
        }
    }
    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setTargetPosition(0);
        frontRightMotor.setTargetPosition(0);
        backLeftMotor.setTargetPosition(0);
        backRightMotor.setTargetPosition(0);

        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;


        ElapsedTime timer = new ElapsedTime();
        moveForwardTime(150);

        turn(-20);
        while (opModeIsActive()){
            time = timer.time(TimeUnit.MILLISECONDS);

            shooter.setPower(shootingHelp.getPID(shooter, 3650));
            if (Math.abs(4000-time)<=50) {
                gate.setPosition(0.3);

                intake.setPower(-1);
            }
            if (Math.abs(14000 - time) <= 50){
                intake.setPower(0);
                shooter.setPower(0);
                break;
            }
        }
        moveForwardTime(800);




    }}