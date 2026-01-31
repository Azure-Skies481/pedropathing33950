package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelModified;

@Autonomous
public class AutoCloseRedPedro extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    double imuAlignAngle;
    private static DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private static Servo gate = null;
    private static FlywheelModified flywheel;

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private IMU imu = null;
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));

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

        class Paths {
            public PathChain goForward;
            public PathChain getBall1;
            public PathChain goBack1;
            public PathChain goGetBall2;
            public PathChain getBall2;
            public PathChain goBack2;
            public PathChain goGetBall3;
            public PathChain getBall3;
            public PathChain goBack3;
            public PathChain goOut;


            public Paths(Follower follower) {
                gate.setPosition(0.36);

                flywheel.setTargetRPM(1300);


                goForward = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(123.000, 123.000),

                                        new Pose(84.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(215), Math.toRadians(215))

                        .build();
                gate.setPosition(0.5);
                intake.setVelocity(0.5);
                sleep(750);
                intake.setVelocity(0);
                gate.setPosition(0.36);
                getBall1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(84.000, 84.000),

                                        new Pose(129.000, 84.000)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(0))

                        .build();

                goBack1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(129.000, 84.000),

                                        new Pose(84.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))

                        .build();
                gate.setPosition(0.5);
                intake.setVelocity(0.5);
                sleep(750);
                intake.setVelocity(0);
                gate.setPosition(0.36);
                goGetBall2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(84.000, 84.000),

                                        new Pose(104.000, 60.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))

                        .build();

                getBall2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(104.000, 60.000),

                                        new Pose(129.000, 60.000)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(0))

                        .build();

                goBack2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(129.000, 60.000),

                                        new Pose(84.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))

                        .build();
                gate.setPosition(0.5);
                intake.setVelocity(0.5);
                sleep(750);
                intake.setVelocity(0);
                gate.setPosition(0.36);

                goGetBall3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(84.000, 84.000),

                                        new Pose(104.000, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))

                        .build();

                getBall3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(104.000, 36.000),

                                        new Pose(129.000, 36.000)
                                )
                        ).setConstantHeadingInterpolation(Math.toRadians(0))

                        .build();

                goBack3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(129.000, 36.000),

                                        new Pose(84.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))

                        .build();
                gate.setPosition(0.5);
                intake.setVelocity(0.5);
                sleep(750);
                intake.setVelocity(0);
                gate.setPosition(0.36);

                goOut = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(84.000, 84.000),

                                        new Pose(105.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))

                        .build();

            }
        }

    }




}
