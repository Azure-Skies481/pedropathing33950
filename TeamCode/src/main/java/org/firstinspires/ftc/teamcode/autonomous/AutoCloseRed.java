package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelModified;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teleop.ShootingHelp;

import java.util.concurrent.TimeUnit;

@Autonomous
@Configurable
public class AutoCloseRed extends LinearOpMode{
    private FlywheelModified flywheel;
    ShootingHelp shootingHelp = new ShootingHelp();

    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private Servo gate = null; //new servo js added
    private double maxspeed = 2800;
    private double feedback = 0.001;
    private double power = 1550;
    private double drivespeed;


    @Sorter(sort = 0) public static double shooterVelocity = 1000;

    private boolean aura = true;
    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

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
            if (Math.abs(error) <= 15 && velocity<=0.5) break;
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
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
    // positive = front. negative = back
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
    // positive = right. negative = left


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
    // positive = left. negative = right


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontleftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class,"backleftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontrightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class,"backrightMotor");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        imuAlignAngle =  imu.getRobotYawPitchRollAngles().getYaw();

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setTargetPosition(0);
        frontRightMotor.setTargetPosition(0);
        backLeftMotor.setTargetPosition(0);
        backRightMotor.setTargetPosition(0);



        intake = hardwareMap.get(DcMotorEx.class, "intakemotor");

        shooter = hardwareMap.get(DcMotorEx.class, "shootermotor");
        gate = hardwareMap.get(Servo.class, "gateServo"); //new servo js added
        flywheel = new FlywheelModified(shooter, telemetry);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        if (isStopRequested()) return;
        telemetry.addData("skibidi", 1);
        telemetry.update();
        // telemetry.addData("a", 2);
        // telemetry.update();
        // telemetry.addData("b", 3);
        //telemetry.update();
        // telemetry.addData("c", 4);
        // telemetry.update();
        //telemetry.addData("d", 5);
        //telemetry.update();
        //telemetry.addData("e", 6);
        //telemetry.update();4

        double actualspeed = shooter.getVelocity();

        moveForward(1450);
        imuAlign();

        int skibidi = 1300;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()){
            time = timer.time(TimeUnit.MILLISECONDS);
            //shooter.setVelocity(shootingHelp.getPID(shooter, 1300));
            flywheel.setTargetRpm(1300);
            flywheel.setShooterOn(true);
            telemetry.addData("Is flywheel on?: ", flywheel.isShooterOn());
            telemetry.addData("Flywheel Speed: ", flywheel.getCurrentRPM());
            telemetry.update();
            if (Math.abs(4000 - time) <= 50) {
                gate.setPosition(0.5);
            }
            if (Math.abs(4400 - time) <= 50){
                intake.setPower(-1);
            }
            if (Math.abs(4410 - time) <= 50){
                intake.setPower(0);
            }
            if (Math.abs(8500 - time) <= 50){
                intake.setPower(-1);
            }
            if (Math.abs(9500 - time) <= 50){
                intake.setPower(0);
                flywheel.setShooterOn(false);
                break;
            }

        }
        turn(90);
        moveForward(900);

    }
}
