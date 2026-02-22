package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelModified;

@Autonomous(name = "New Red Close", group = "Autonomous", preselectTeleOp = "A HORS OFFICIAL ⭐")
@Configurable
public class NewRedClose extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private enum AutoState { IDLE, WAIT_FOR_SHOOTER, RUNNING_PATH, WAIT_FIRST_SHOOT, CLOSED_INTAKE_SEQUENCE, PRE_ACTION, INTAKE_RUN, FINISHED }
    private AutoState state = AutoState.IDLE;

    private int currentPathIndex = 0;
    private int nextPathIndex = -1;

    private Timer intakeTimer;
    private Timer timedIntakeTimer;
    private boolean timedIntakeActive = false;

    private Timer preActionTimer;
    private Timer poseWaitTimer;
    private Timer firstShootWaitTimer;

    private long shooterWaitStartMs = -1;

    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;

    private Servo gateServo;
    private boolean gateClosed = false;

    private FlywheelModified flywheel;
    private VoltageSensor voltageSensor;

    private long autoStartMs = -1;
    private boolean shutdownDone = false;

    private boolean preActionTimerStarted = false;

    // ========================================
    // TIMING PARAMETERS
    // ========================================
    @Sorter(sort = 0)
    public static double INTAKE_RUN_SECONDS = 0.8;
    @Sorter(sort = 1)
    public static double TIMED_INTAKE_SECONDS = 0.6;
    @Sorter(sort = 2)
    public static double PRE_ACTION_FIRST_SHOOT_WAIT_SECONDS = 0.2;
    @Sorter(sort = 3)
    public static double PRE_ACTION_WAIT_SECONDS = 0.65;
    @Sorter(sort = 4)
    public static double PRE_ACTION_MAX_POSE_WAIT_SECONDS = 0.85;
    @Sorter(sort = 5)
    public static long SHOOTER_WAIT_TIMEOUT_MS = 1100L;

    // ========================================
    // INTAKE POWER SETTINGS
    // ========================================
    @Sorter(sort = 10)
    public static double INTAKE_ON_POWER = -0.75;
    @Sorter(sort = 11)
    public static double SHOOT_POSE_INTAKE_POWER = -1.0;
    @Sorter(sort = 12)
    public static double CLOSED_INTAKE_POWER = -0.67;
    @Sorter(sort = 13)
    public static double CLOSED_INTAKE_TOLERANCE_IN = 10.0;

    // ========================================
    // TOLERANCE SETTINGS
    // ========================================
    @Sorter(sort = 20)
    public static double START_POSE_TOLERANCE_IN = 5.0;

    // ========================================
    // GATE SETTINGS (HORS replica: open=0.10, closed=0.30)
    // ========================================
    @Sorter(sort = 30)
    public static double GATE_OPEN = 0.3;
    @Sorter(sort = 31)
    public static double GATE_CLOSED = 0.1;
    @Sorter(sort = 32)
    public static double GATE_OPEN_TOLERANCE_IN = 10.0;
    @Sorter(sort = 33)
    public static double GATE_CLOSE_TOLERANCE_IN = 10.0;

    // ========================================
    // PATH POSES - START POSITION & SHOOT/ALIGN HEADINGS
    // ========================================
    @Sorter(sort = 100)
    public static double START_X = 122.0;
    @Sorter(sort = 101)
    public static double START_Y = 122.0;
    @Sorter(sort = 102)
    public static double START_HEADING = 45;
    @Sorter(sort = 110)
    public static double SHOOT_POSE_X = 84;
    @Sorter(sort = 111)
    public static double SHOOT_POSE_Y = 84;
    @Sorter(sort = 112)
    public static double SHOOT_HEADING_INITIAL = 45;
    @Sorter(sort = 113)
    public static double SHOOT_HEADING_FIRST3 = 45;
    @Sorter(sort = 114)
    public static double SHOOT_SECOND3_HEADING = 45;
    @Sorter(sort = 115)
    public static double SHOOT_FINAL_HEADING = 45;
    @Sorter(sort = 120)
    public static double COLLECT_FIRST3_X = 125.0;
    @Sorter(sort = 121)
    public static double COLLECT_FIRST3_Y = 90.0;
    @Sorter(sort = 122)
    public static double COLLECT_FIRST3_HEADING = 0;
    @Sorter(sort = 140)
    public static double ALIGN_SECOND3_X = 95.0;
    @Sorter(sort = 141)
    public static double ALIGN_SECOND3_Y = 64.0;
    @Sorter(sort = 142)
    public static double ALIGN_SECOND3_HEADING = 0;
    @Sorter(sort = 150)
    public static double COLLECT_SECOND3_X = 125.0;
    @Sorter(sort = 151)
    public static double COLLECT_SECOND3_Y = 64;
    @Sorter(sort = 152)
    public static double COLLECT_SECOND3_HEADING = 0;
    @Sorter(sort = 160)
    public static double ALIGN_THIRD3_X = 95.0;
    @Sorter(sort = 161)
    public static double ALIGN_THIRD3_Y = 41.0;
    @Sorter(sort = 162)
    public static double ALIGN_THIRD3_HEADING = 0;
    @Sorter(sort = 170)
    public static double COLLECT_THIRD3_X = 125.0;
    @Sorter(sort = 171)
    public static double COLLECT_THIRD3_Y = 41.0;
    @Sorter(sort = 172)
    public static double COLLECT_THIRD3_HEADING = 0;
    @Sorter(sort = 180)
    public static double MOVE_RP_X = 100.0;
    @Sorter(sort = 181)
    public static double MOVE_RP_Y = 80.0;
    @Sorter(sort = 182)
    public static double MOVE_RP_HEADING = 45;

    private static final double AUTO_SHOOTER_RPM = 3900;

    public NewRedClose() {}

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING)));

        intakeTimer = new Timer();
        timedIntakeTimer = new Timer();
        preActionTimer = new Timer();
        poseWaitTimer = new Timer();
        firstShootWaitTimer = new Timer();

        nextPathIndex = -1;
        timedIntakeActive = false;
        preActionTimerStarted = false;

        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Failed to map shooter: " + e.getMessage());
        }
        try {
            shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            panelsTelemetry.debug("Init", "Shooter2 initialized");
        } catch (Exception e) {
            shooterMotor2 = null;
            panelsTelemetry.debug("Init", "Shooter2 not found: " + e.getMessage());
        }
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setPower(0.0);
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Intake mapping failed: " + e.getMessage());
        }
        try {
            gateServo = hardwareMap.get(Servo.class, "gateServo");
            if (gateServo != null) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Gate servo mapping failed: " + e.getMessage());
        }
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        try {
            if (shooterMotor != null) {
                flywheel = new FlywheelModified(shooterMotor, shooterMotor2, telemetry, voltageSensor);
                flywheel.setShooterOn(false);
                flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Init", "Flywheel init error: " + e.getMessage());
        }

        panelsTelemetry.debug("Status", "Initialized (shooter OFF until start())");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // keep flywheel off
    }

    @Override
    public void start() {
        autoStartMs = System.currentTimeMillis();

        if (flywheel != null) {
            flywheel.setShooterOn(true);
            flywheel.setTargetRPM(AUTO_SHOOTER_RPM);
        }

        shooterWaitStartMs = System.currentTimeMillis();
        state = AutoState.WAIT_FOR_SHOOTER;
    }

    @Override
    public void loop() {
        follower.update();
        long nowMs = System.currentTimeMillis();

        if (flywheel != null) {
            flywheel.update();
        }

        runStateMachine(nowMs);

        // Gate updates except during WAIT_FIRST_SHOOT (keep closed then)
        if (state != AutoState.WAIT_FIRST_SHOOT) {
            updateGate();
        }

        double elapsedSec = (autoStartMs > 0) ? (nowMs - autoStartMs) / 1000.0 : 0.0;
        panelsTelemetry.debug("Elapsed(s)", String.format("%.2f", elapsedSec));
        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("PathIdx", currentPathIndex);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        if (flywheel != null) {
            panelsTelemetry.debug("Fly RPM", String.format("%.1f", flywheel.getCurrentRPM()));
            panelsTelemetry.debug("Fly Target", String.format("%.1f", flywheel.getTargetRPM()));
            panelsTelemetry.debug("Fly On", flywheel.isShooterOn());
        }
        if (intakeMotor != null) {
            panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
        }
        panelsTelemetry.debug("DistToShootPose", String.format("%.2f", distanceToShootPose()));
        panelsTelemetry.debug("GateClosed", String.valueOf(gateClosed));

        panelsTelemetry.update(telemetry);

        if (state == AutoState.FINISHED && !shutdownDone) {
            resetToInitState();
            shutdownDone = true;
        }
    }

    @Override
    public void stop() {
        resetToInitState();
        state = AutoState.FINISHED;
    }

    private void resetToInitState() {
        if (flywheel != null) {
            flywheel.setShooterOn(false);
            flywheel.setTargetRPM(0.0);
            flywheel.update();
        }
        stopIntake();
        if (gateServo != null) {
            gateServo.setPosition(GATE_CLOSED);
            gateClosed = true;
        }
    }

    private void startIntake() { startIntake(INTAKE_ON_POWER); }
    private void startIntake(double power) {
        try { if (intakeMotor != null) intakeMotor.setPower(power); }
        catch (Exception e) { panelsTelemetry.debug("Intake", "startIntake error: " + e.getMessage()); }
    }
    private void stopIntake() {
        try { if (intakeMotor != null) intakeMotor.setPower(0.0); }
        catch (Exception e) { panelsTelemetry.debug("Intake", "stopIntake error: " + e.getMessage()); }
    }

    private boolean endsAtShoot(int pathIndex) {
        return pathIndex == 1 || pathIndex == 3 || pathIndex == 6 || pathIndex == 9;
    }

    private double distanceToShootPose() {
        try {
            Pose p = follower.getPose();
            double dx = p.getX() - SHOOT_POSE_X;
            double dy = p.getY() - SHOOT_POSE_Y;
            return Math.hypot(dx, dy);
        } catch (Exception e) {
            return Double.POSITIVE_INFINITY;
        }
    }

    private void startPath(int idx) {
        // paths 1..10 only now
        if (idx < 1 || idx > 10) {
            currentPathIndex = 0;
            state = AutoState.FINISHED;
            return;
        }

        startIntake(INTAKE_ON_POWER);

        if (idx == 3 || idx == 6 || idx == 9) { // back-to-shoot legs
            timedIntakeTimer.resetTimer();
            timedIntakeActive = true;
            panelsTelemetry.debug("TimedIntake", "Started timed intake for path " + idx);
        }

        switch (idx) {
            case 1: follower.followPath(paths.startToShoot); break;
            case 2: follower.followPath(paths.collectFirst3); break;
            case 3: follower.followPath(paths.backToShootFirst3); break;
            case 4: follower.followPath(paths.alignToCollectSecond3); break;
            case 5: follower.followPath(paths.collectSecond3); break;
            case 6: follower.followPath(paths.backToShootSecond3); break;
            case 7: follower.followPath(paths.alignToCollectThird3); break;
            case 8: follower.followPath(paths.collectThird3); break;
            case 9: follower.followPath(paths.backToShootThird3); break;
            case 10: follower.followPath(paths.moveForRP); break;
            default: break;
        }

        currentPathIndex = idx;
        state = AutoState.RUNNING_PATH;
    }

    private void runStateMachine(long nowMs) {
        if (timedIntakeActive) {
            if (timedIntakeTimer.getElapsedTimeSeconds() >= TIMED_INTAKE_SECONDS) {
                startIntake(INTAKE_ON_POWER);
                timedIntakeActive = false;
                panelsTelemetry.debug("TimedIntake", "Timed intake done; continuing at travel power");
            } else {
                panelsTelemetry.debug("TimedIntake", String.format("remaining=%.2fs", TIMED_INTAKE_SECONDS - timedIntakeTimer.getElapsedTimeSeconds()));
            }
        }

        switch (state) {
            case WAIT_FOR_SHOOTER:
                boolean atTarget = (flywheel != null && flywheel.isAtTarget());
                long elapsed = (shooterWaitStartMs < 0) ? 0 : (System.currentTimeMillis() - shooterWaitStartMs);
                if (atTarget || elapsed >= SHOOTER_WAIT_TIMEOUT_MS) {
                    startPath(1);
                }
                break;

            case RUNNING_PATH:
                if (!follower.isBusy()) {
                    int finished = currentPathIndex;

                    if (endsAtShoot(finished)) {
                        nextPathIndex = finished + 1;
                        if (finished == 1) {
                            stopIntake();
                            if (gateServo != null && !gateClosed) {
                                gateServo.setPosition(GATE_CLOSED);
                                gateClosed = true;
                            }
                            firstShootWaitTimer.resetTimer();
                            state = AutoState.WAIT_FIRST_SHOOT;
                            panelsTelemetry.debug("WAIT_FIRST_SHOOT", "Path 1 done, waiting " + PRE_ACTION_FIRST_SHOOT_WAIT_SECONDS + "s");
                        } else {
                            state = AutoState.CLOSED_INTAKE_SEQUENCE;
                        }
                    } else {
                        int next = finished + 1;
                        if (next > 10) state = AutoState.FINISHED;
                        else startPath(next);
                    }
                }
                break;

            case WAIT_FIRST_SHOOT:
                double waitElapsed = firstShootWaitTimer.getElapsedTimeSeconds();
                panelsTelemetry.debug("WAIT_FIRST_SHOOT", String.format("remaining=%.2fs", PRE_ACTION_FIRST_SHOOT_WAIT_SECONDS - waitElapsed));
                if (waitElapsed >= PRE_ACTION_FIRST_SHOOT_WAIT_SECONDS) {
                    state = AutoState.CLOSED_INTAKE_SEQUENCE;
                    panelsTelemetry.debug("WAIT_FIRST_SHOOT", "Wait complete, entering CLOSED_INTAKE_SEQUENCE");
                }
                break;

            case CLOSED_INTAKE_SEQUENCE:
                double distPre = distanceToShootPose();
                if (distPre <= CLOSED_INTAKE_TOLERANCE_IN) {
                    startIntake(CLOSED_INTAKE_POWER);
                }
                if (distPre <= START_POSE_TOLERANCE_IN) {
                    poseWaitTimer.resetTimer();
                    preActionTimerStarted = false;
                    state = AutoState.PRE_ACTION;
                }
                break;

            case PRE_ACTION:
                if (!preActionTimerStarted) {
                    double dist = distanceToShootPose();
                    if (dist <= START_POSE_TOLERANCE_IN || poseWaitTimer.getElapsedTimeSeconds() >= PRE_ACTION_MAX_POSE_WAIT_SECONDS) {
                        preActionTimer.resetTimer();
                        preActionTimerStarted = true;
                        panelsTelemetry.debug("PRE_ACTION", "Starting PRE_ACTION timer (dist=" + String.format("%.2f", dist) + ")");
                    } else {
                        panelsTelemetry.debug("PRE_ACTION", "Waiting for pose (dist=" + String.format("%.2f", dist) + ")");
                        break;
                    }
                } else {
                    if (preActionTimer.getElapsedTimeSeconds() >= PRE_ACTION_WAIT_SECONDS) {
                        startIntake(SHOOT_POSE_INTAKE_POWER);
                        intakeTimer.resetTimer();
                        state = AutoState.INTAKE_RUN;
                    } else {
                        panelsTelemetry.debug("PRE_ACTION", String.format("settle remaining=%.2fs", PRE_ACTION_WAIT_SECONDS - preActionTimer.getElapsedTimeSeconds()));
                    }
                }
                break;

            case INTAKE_RUN:
                if (intakeTimer.getElapsedTimeSeconds() >= INTAKE_RUN_SECONDS) {
                    startIntake(INTAKE_ON_POWER);
                    if (flywheel != null) flywheel.setTargetRPM(0.95 * AUTO_SHOOTER_RPM);
                    if (nextPathIndex > 0 && nextPathIndex <= 10) {
                        startPath(nextPathIndex);
                        nextPathIndex = -1;
                    } else {
                        state = AutoState.FINISHED;
                    }
                }
                break;

            case FINISHED:
            case IDLE:
            default:
                break;
        }
    }

    private void updateGate() {
        try {
            double dist = distanceToShootPose();
            if (dist <= GATE_OPEN_TOLERANCE_IN && gateServo != null && gateClosed) {
                gateServo.setPosition(GATE_OPEN);
                gateClosed = false;
                panelsTelemetry.debug("Gate", "Opened (dist=" + String.format("%.2f", dist) + ")");
            } else if (dist >= GATE_CLOSE_TOLERANCE_IN && gateServo != null && !gateClosed) {
                gateServo.setPosition(GATE_CLOSED);
                gateClosed = true;
                panelsTelemetry.debug("Gate", "Closed (dist=" + String.format("%.2f", dist) + ")");
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Gate", "updateGate error: " + e.getMessage());
        }
    }

    public static class Paths {
        public PathChain startToShoot;
        public PathChain collectFirst3;
        public PathChain backToShootFirst3;
        public PathChain alignToCollectSecond3;
        public PathChain collectSecond3;
        public PathChain backToShootSecond3;
        public PathChain alignToCollectThird3;
        public PathChain collectThird3;
        public PathChain backToShootThird3;
        public PathChain moveForRP;

        private static final double TURN_EPS = 0.01;

        private PathChain buildTurnThenDrive(Follower follower, Pose start, Pose end, double startHeadingDeg, double targetHeadingDeg) {
            Pose turnNudge = new Pose(start.getX() + TURN_EPS, start.getY() + TURN_EPS);
            return follower.pathBuilder()
                    .addPath(new BezierLine(start, turnNudge))
                    .setLinearHeadingInterpolation(Math.toRadians(startHeadingDeg), Math.toRadians(targetHeadingDeg))
                    .addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(Math.toRadians(targetHeadingDeg), Math.toRadians(targetHeadingDeg))
                    .build();
        }

        public Paths(Follower follower) {
            startToShoot = buildTurnThenDrive(
                    follower,
                    new Pose(START_X, START_Y),
                    new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                    START_HEADING,
                    SHOOT_HEADING_INITIAL
            );

            collectFirst3 = buildTurnThenDrive(
                    follower,
                    new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                    new Pose(COLLECT_FIRST3_X, COLLECT_FIRST3_Y),
                    SHOOT_HEADING_INITIAL,
                    COLLECT_FIRST3_HEADING
            );

            backToShootFirst3 = buildTurnThenDrive(
                    follower,
                    new Pose(COLLECT_FIRST3_X, COLLECT_FIRST3_Y),
                    new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                    COLLECT_FIRST3_HEADING,
                    SHOOT_HEADING_FIRST3
            );

            alignToCollectSecond3 = buildTurnThenDrive(
                    follower,
                    new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                    new Pose(ALIGN_SECOND3_X, ALIGN_SECOND3_Y),
                    SHOOT_HEADING_FIRST3,
                    ALIGN_SECOND3_HEADING
            );

            collectSecond3 = buildTurnThenDrive(
                    follower,
                    new Pose(ALIGN_SECOND3_X, ALIGN_SECOND3_Y),
                    new Pose(COLLECT_SECOND3_X, COLLECT_SECOND3_Y),
                    ALIGN_SECOND3_HEADING,
                    COLLECT_SECOND3_HEADING
            );

            backToShootSecond3 = buildTurnThenDrive(
                    follower,
                    new Pose(COLLECT_SECOND3_X, COLLECT_SECOND3_Y),
                    new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                    COLLECT_SECOND3_HEADING,
                    SHOOT_SECOND3_HEADING
            );

            alignToCollectThird3 = buildTurnThenDrive(
                    follower,
                    new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                    new Pose(ALIGN_THIRD3_X, ALIGN_THIRD3_Y),
                    SHOOT_SECOND3_HEADING,
                    ALIGN_THIRD3_HEADING
            );

            collectThird3 = buildTurnThenDrive(
                    follower,
                    new Pose(ALIGN_THIRD3_X, ALIGN_THIRD3_Y),
                    new Pose(COLLECT_THIRD3_X, COLLECT_THIRD3_Y),
                    ALIGN_THIRD3_HEADING,
                    COLLECT_THIRD3_HEADING
            );

            backToShootThird3 = buildTurnThenDrive(
                    follower,
                    new Pose(COLLECT_THIRD3_X, COLLECT_THIRD3_Y),
                    new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                    COLLECT_THIRD3_HEADING,
                    SHOOT_FINAL_HEADING
            );

            moveForRP = buildTurnThenDrive(
                    follower,
                    new Pose(SHOOT_POSE_X, SHOOT_POSE_Y),
                    new Pose(MOVE_RP_X, MOVE_RP_Y),
                    SHOOT_FINAL_HEADING,
                    MOVE_RP_HEADING
            );
        }
    }
}