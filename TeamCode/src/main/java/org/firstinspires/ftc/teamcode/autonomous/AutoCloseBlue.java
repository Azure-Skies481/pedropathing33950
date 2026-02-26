package org.firstinspires.ftc.teamcode.autonomous;



import static org.firstinspires.ftc.teamcode.teleop.MecanumTeleop.feedback;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.ShootingHelp;
import org.firstinspires.ftc.teamcode.teleop.ShootingHelp.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous (group = "test")
public class AutoCloseBlue extends LinearOpMode {

    private ShootingHelp shootingHelp = new ShootingHelp();
    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private Servo gate = null; //new servo js added
    //private double maxspeed = 2800;
    //private double feedback = 0.001;
    private boolean wasPressedLastFrame = false;
    private double time;
    private boolean gateWasPressedLastFrame = false; //Satoru...
    private boolean gateOpen = false; //Suguru...
    private boolean shooterToggle = false;
    private double power = 1550;
    private double drivespeed;
    private double maxspeed = 2800;

    private boolean keepShooting = true;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private IMU imu = null;
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
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
            if (Math.abs(error) <= 15 && velocity <= 0.3) break;
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public void strafe(double amount, boolean left) {
        resetEncoders();
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

    double imuAlignAngle;
    public void imuAlign() {
        double timeout = 0.5;
        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();

        while (opModeIsActive()) {
            double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = imuAlignAngle - imuAngle;
            double power = 0.02 * error;

            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);

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

        imuAlignAngle = imu.getRobotYawPitchRollAngles().getYaw();
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setTargetPosition(0);
        frontRightMotor.setTargetPosition(0);
        backLeftMotor.setTargetPosition(0);
        backRightMotor.setTargetPosition(0);

        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        double actualspeed = shooter.getVelocity();

        moveForward(1200);


        ElapsedTime timer = new ElapsedTime();
        int skibidi = 1300;
        while (opModeIsActive()){
            time = timer.time(TimeUnit.MILLISECONDS);



            shooter.setPower(shootingHelp.getPID(shooter, skibidi));

            if (Math.abs(4000 - time) <= 50) {
                gate.setPosition(0.5);
            }
            if (Math.abs(4400 - time) <= 50){
                intake.setPower(-1);
            }
            if (Math.abs(4410- time) <= 50){
                intake.setPower(0);
            }
            if (Math.abs(8500 - time) <= 50){
                intake.setPower(-1);
            }
            if (Math.abs(9500 - time) <= 50){
                intake.setPower(0);
                shooter.setPower(0);
                break;
            }
        }
        turn(-90);
        moveForward(900);

    }
}
