package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.oysterbay.base.BulkReads;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Holders;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Intake;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.TagVision;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.TagVote;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Tipper;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Trap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.function.DoubleConsumer;

/**
 * Shared setup and building-block procedures for the OB autonomous programs.
 *
 * Subclasses provide the shooter RPM and the route in {@link #runAuto()}; everything else
 * (hardware init, camera, tag scan, shooting sequences, timed drives) lives here.
 *
 * Drive convention: the drivetrain is set up with the LEFT side reversed, as the autos were tuned.
 * {@link #driveForward(double)} hides the resulting sign flip so positive power = forward.
 */
public abstract class OBAutoBase extends LinearOpMode {

    // =========================
    // Tag IDs you CARE about
    // =========================
    protected static final int TAG1_ID = 21; // Plan 1
    protected static final int TAG2_ID = 22; // Plan 2
    private static final int MIN_SEEN_FRAMES = 2;

    // =========================
    // Camera
    // =========================
    private static final String CAM_SERVO_NAME = "camServo";
    private static final double CAM_LEFT_POS = 0.0;
    private static final double CAM_CENTER_POS = 0.5;

    private static final boolean LOCK_CAMERA_EXPOSURE = false;
    private static final Float DECIMATION = 2.0f;

    // =========================
    // Shooting timing
    // =========================
    private static final double AT_SPEED_TOL_RPM = 150.0;
    private static final double SPINUP_TIMEOUT_SEC = 2.0; // fail-safe so you still fire if sensor noise
    private static final long FIRE_HOLD_MS = 350;
    private static final long SPIN_DOWN_SETTLE_MS = 150;

    private static final long STAGE_SETTLE_MS = 600;   // after opening a holder / feeding the intake
    private static final long TRAP_PUSH_MS = 1200;
    private static final long TRAP_PAUSE_MS = 1000;

    private static final long PINPOINT_CALIBRATE_MS = 350;

    // =========================
    // Move-to-position (Pinpoint)
    // =========================
    private static final double POS_TOL_MM = 15.0;
    private static final double MOVE_TIMEOUT_SEC = 4.0;
    private static final double MOVE_FWD_POWER = 0.35;
    private static final double MOVE_STRAFE_POWER = 0.35;
    // If the robot strafes away from the Y target, flip this
    private static final boolean PINPOINT_Y_INCREASES_WHEN_LEFT = true;

    /** Where the next ball comes from before it is pushed into the shooter. */
    protected enum ShotSource { INTAKE, LEFT_HOLDER, RIGHT_HOLDER, TRAP }

    protected MecanumDrivetrain drive;
    protected Shooter shooter;
    protected Intake intake;
    protected Trap trap;
    protected Tipper tipper;
    protected Holders holders;
    protected Odometry odometry;
    protected TagVision vision;
    private Servo camServo;

    /** Flywheel RPM this auto shoots at. */
    protected abstract double shooterTargetRpm();

    /** The route: everything that happens after START. */
    protected abstract void runAuto();

    @Override
    public final void runOpMode() {
        initRobot();

        telemetry.addLine("Init complete. Press START.");
        telemetry.addData("Pinpoint Status", odometry.status());
        telemetry.addData("Shooter Target RPM", "%.0f", shooterTargetRpm());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runAuto();

        shooter.stop();
        vision.close();
    }

    // =========================
    // Init
    // =========================
    private void initRobot() {
        BulkReads.auto(hardwareMap);

        camServo = hardwareMap.get(Servo.class, CAM_SERVO_NAME);
        camServo.setPosition(CAM_CENTER_POS);

        drive   = new MecanumDrivetrain(hardwareMap, MecanumDrivetrain.ReversedSide.LEFT);
        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        trap    = new Trap(hardwareMap);
        tipper  = new Tipper(hardwareMap);
        holders = Holders.forAuto(hardwareMap);

        initOdometry();
        initVision();
    }

    private void initOdometry() {
        odometry = new Odometry(hardwareMap);
        odometry.applyRobotGeometry();
        odometry.resetPosAndIMU();
        sleep(PINPOINT_CALIBRATE_MS);
        odometry.update();
    }

    private void initVision() {
        vision = new TagVision(hardwareMap, DECIMATION);
        vision.waitForStreaming(this::isStopRequested, () -> { sleep(10); idle(); });
        if (LOCK_CAMERA_EXPOSURE) vision.lockExposure();

        // Dashboard camera stream + merged telemetry
        vision.startDashboardStream();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    // =========================
    // Loop helpers
    // =========================

    /** Run {@code body(elapsedSeconds)} every loop for {@code seconds}, then update telemetry. */
    protected void runFor(double seconds, DoubleConsumer body) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds) {
            body.accept(t.seconds());
            telemetry.update();
            idle();
        }
    }

    protected void say(String line) {
        telemetry.addLine(line);
        telemetry.update();
    }

    // =========================
    // Drive procedures
    // =========================

    /** Straight drive, positive = robot forward. */
    protected void driveForward(double power) {
        // With the left side reversed, negative raw power moves the robot forward
        drive.setAll(-power);
    }

    protected void stopDrive() {
        drive.stop();
    }

    /** Drive straight for a fixed time while showing odometry. */
    protected void driveForwardFor(double power, double seconds, String phase) {
        driveForward(power);
        runFor(seconds, t -> {
            odometry.update();
            telemetry.addData("Phase", phase);
            telemetry.addData("t", "%.2f / %.2f", t, seconds);
            odometry.addPoseTelemetry(telemetry, true);
        });
        stopDrive();
    }

    /** Mecanum drive (forward, strafe, turn) for a fixed time, then stop. */
    protected void driveFor(double forward, double strafe, double turn, long ms) {
        drive.drive(forward, strafe, turn);
        sleep(ms);
        stopDrive();
    }

    /** Drive at constant power while running the intake (time-based). */
    protected void pickUp(double drivePower, double intakePower, double seconds) {
        driveForward(drivePower);
        intake.setPower(intakePower);
        runFor(seconds, t -> {});
        stopDrive();
        intake.stop();
    }

    /** Simple X-then-Y move to a field position (mm) using Pinpoint. */
    protected void moveToPosition(double targetXmm, double targetYmm) {
        ElapsedTime timeout = new ElapsedTime();

        while (opModeIsActive() && timeout.seconds() < MOVE_TIMEOUT_SEC) {
            Pose2D pose = odometry.update();
            if (pose == null) {
                say("Pose is null");
                idle();
                continue;
            }

            double xErr = targetXmm - pose.getX(DistanceUnit.MM);
            double yErr = targetYmm - pose.getY(DistanceUnit.MM);
            boolean atX = Math.abs(xErr) <= POS_TOL_MM;
            boolean atY = Math.abs(yErr) <= POS_TOL_MM;
            if (atX && atY) break;

            double forward = 0.0;
            double strafe = 0.0;
            if (!atX) {
                forward = Math.copySign(MOVE_FWD_POWER, xErr);
            } else {
                double desired = Math.copySign(MOVE_STRAFE_POWER, yErr);
                strafe = PINPOINT_Y_INCREASES_WHEN_LEFT ? desired : -desired;
            }
            drive.drive(forward, strafe, 0.0);

            telemetry.addData("Target", "%.1f, %.1f", targetXmm, targetYmm);
            telemetry.addData("Err", "%.1f, %.1f", xErr, yErr);
            odometry.addPoseTelemetry(telemetry, false);
            telemetry.update();
            idle();
        }

        stopDrive();
    }

    // =========================
    // Vision procedures
    // =========================
    protected void pointCameraLeft() {
        camServo.setPosition(CAM_LEFT_POS);
    }

    /**
     * Count tag sightings for {@code seconds} (robot should be still) and lock a decision.
     *
     * @return TAG1_ID, TAG2_ID, or null if neither was seen confidently (default plan)
     */
    protected Integer scanForTag(double seconds) {
        TagVote vote = new TagVote(TAG1_ID, TAG2_ID, MIN_SEEN_FRAMES);

        runFor(seconds, t -> {
            odometry.update();
            List<AprilTagDetection> dets = vision.detections();
            vote.record(dets);

            telemetry.addData("Phase", "STOPPED Scan");
            telemetry.addData("Scan t", "%.2f / %.2f", t, seconds);
            telemetry.addData("Detections", dets.size());
            telemetry.addData("Seen Tag1 (" + TAG1_ID + ")", vote.seen1());
            telemetry.addData("Seen Tag2 (" + TAG2_ID + ")", vote.seen2());
            odometry.addPoseTelemetry(telemetry, true);
        });

        Integer decision = vote.decide();
        telemetry.addData("Decision (locked)", vote.label(decision));
        telemetry.update();
        return decision;
    }

    // =========================
    // Shooting procedures
    // =========================

    /** Shoot all balls in the order that matches the scanned tag. */
    protected void runShootingPlan(Integer tagId) {
        if (tagId != null && tagId == TAG1_ID) {
            say("Running TAG 1 plan.");
            shootSequence(ShotSource.INTAKE, ShotSource.LEFT_HOLDER, ShotSource.RIGHT_HOLDER, ShotSource.TRAP);
        } else if (tagId != null && tagId == TAG2_ID) {
            say("Running TAG 2 plan.");
            shootSequence(ShotSource.RIGHT_HOLDER, ShotSource.INTAKE, ShotSource.LEFT_HOLDER, ShotSource.TRAP);
        } else {
            say("Running TAG 3 DEFAULT plan.");
            shootSequence(ShotSource.RIGHT_HOLDER, ShotSource.LEFT_HOLDER, ShotSource.INTAKE, ShotSource.TRAP);
        }
    }

    protected void shootSequence(ShotSource... order) {
        for (ShotSource source : order) {
            shootFrom(source);
        }
    }

    /** Release one ball from {@code source}, push it with the trap, and fire it. */
    protected void shootFrom(ShotSource source) {
        stage(source);
        pushWithTrap();
        shootBall();
    }

    private void stage(ShotSource source) {
        switch (source) {
            case LEFT_HOLDER:
                holders.openLeft();
                sleep(STAGE_SETTLE_MS);
                break;
            case RIGHT_HOLDER:
                holders.openRight();
                sleep(STAGE_SETTLE_MS);
                break;
            case INTAKE:
                intake.feed();
                sleep(STAGE_SETTLE_MS);
                intake.stop();
                break;
            case TRAP:
                break; // ball already sitting in the trap
        }
    }

    /** Two trap pushes with a pause between them. */
    protected void pushWithTrap() {
        trap.feed();
        sleep(TRAP_PUSH_MS);
        trap.stop();
        sleep(TRAP_PAUSE_MS);
        trap.feed();
        sleep(TRAP_PUSH_MS);
        trap.stop();
    }

    /** Spin up, fire the tipper once, spin down. */
    protected void shootBall() {
        spinUp();

        tipper.fire();
        sleep(FIRE_HOLD_MS);
        tipper.rest();

        shooter.stop();
        sleep(SPIN_DOWN_SETTLE_MS);
    }

    /** Command the target RPM and wait until both wheels are at speed (or the timeout passes). */
    private void spinUp() {
        double target = shooterTargetRpm();
        shooter.setRpm(target);

        ElapsedTime spin = new ElapsedTime();
        while (opModeIsActive() && spin.seconds() < SPINUP_TIMEOUT_SEC) {
            double lRpm = shooter.leftRpm();
            double rRpm = shooter.rightRpm();
            boolean atSpeed = Shooter.bothWithin(lRpm, rRpm, target, AT_SPEED_TOL_RPM);

            telemetry.addData("Shooter Target", "%.0f", target);
            telemetry.addData("Shooter L RPM", "%.0f", lRpm);
            telemetry.addData("Shooter R RPM", "%.0f", rRpm);
            telemetry.addData("At speed?", atSpeed);
            telemetry.update();

            if (atSpeed) break;
            idle();
        }
    }
}
