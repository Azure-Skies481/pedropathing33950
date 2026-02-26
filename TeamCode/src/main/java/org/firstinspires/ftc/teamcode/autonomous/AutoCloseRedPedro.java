package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelModified;

@Autonomous (group = "test")
public class AutoCloseRedPedro extends LinearOpMode {
    private Follower follower;

    private DcMotorEx intake;
    private DcMotorEx shooter;
    private DcMotorEx shooter2;
    private Servo gate;
    private FlywheelModified flywheel;
    private VoltageSensor battery;

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
    private final Pose scoringPose = new Pose(84,84,Math.toRadians(45));
    private final Pose startingPose = new Pose(123,123,Math.toRadians(45));
    private final Pose pickup1Pose = new Pose(100, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(100, 60, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(100, 35, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class,"backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRight");
        backRightMotor  = hardwareMap.get(DcMotorEx.class,"backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        gate = hardwareMap.get(Servo.class, "gateServo");
        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = initFollower();
        if (follower == null) {
            telemetry.addLine("Follower not initialized");
            telemetry.update();
            waitForStart();
            return;
        }

        flywheel = new FlywheelModified(shooter, telemetry, battery);

        PathChain goForward = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(123.000, 123.000), new Pose(84.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(215))
                .build();

        PathChain getBall1 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(129.000, 84.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain goBack1 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(129.000, 84.000), new Pose(84.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))
                .build();

        PathChain goGetBall2 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(104.000, 60.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))
                .build();

        PathChain getBall2 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(104.000, 60.000), new Pose(129.000, 60.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain goBack2 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(129.000, 60.000), new Pose(84.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))
                .build();

        PathChain goGetBall3 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(104.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))
                .build();

        PathChain getBall3 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(104.000, 36.000), new Pose(129.000, 36.000)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain goBack3 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(129.000, 36.000), new Pose(84.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))
                .build();

        PathChain goOut = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(84.000, 84.000), new Pose(105.000, 84.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        gate.setPosition(0.3);
        flywheel.setTargetRPM(1300);
        flywheel.setShooterOn(true);

        long spinupStart = System.currentTimeMillis();
        while (opModeIsActive()
                && !flywheel.isAtSpeed()
                && (System.currentTimeMillis() - spinupStart < 2000)) {
            flywheel.update();
            telemetry.addData("RPM", flywheel.getCurrentRPM());
            telemetry.addData("AtSpeed", flywheel.isAtSpeed());
            telemetry.update();
        }

        runPath(goForward, "goForward");
        gate.setPosition(0);
        intake.setPower(0.5);
        sleep(750);
        intake.setPower(0);
        gate.setPosition(0.3);

        runPath(getBall1, "getBall1");
        runPath(goBack1, "goBack1");
        gate.setPosition(0);
        intake.setPower(0.5);
        sleep(750);
        intake.setPower(0);
        gate.setPosition(0.3);

        runPath(goGetBall2, "goGetBall2");
        runPath(getBall2, "getBall2");
        runPath(goBack2, "goBack2");
        gate.setPosition(0);
        intake.setPower(0.5);
        sleep(750);
        intake.setPower(0);
        gate.setPosition(0.3);

        runPath(goGetBall3, "goGetBall3");
        runPath(getBall3, "getBall3");
        runPath(goBack3, "goBack3");
        gate.setPosition(0);
        intake.setPower(0.5);
        sleep(750);
        intake.setPower(0);
        gate.setPosition(0.3);

        runPath(goOut, "goOut");

        flywheel.setShooterOn(false);
    }

    private void runPath(PathChain chain, String stage) {
        follower.followPath(chain);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            flywheel.update();
            telemetry.addData("stage", stage);
            telemetry.addData("RPM", flywheel.getCurrentRPM());
            telemetry.update();
        }
    }

    private Follower initFollower() {
        return null; // drivetrain and constraints
    }
}