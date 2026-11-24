// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/ShooterDistanceTuner.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/*
 * Shooter Distance Tuner: fills in the DISTANCE -> RPM table for LimelightTagLockAutoRpm
 * by having you take test shots from a list of distances.
 *
 * DISTANCE here is measured along the floor, from the spot directly under the center of the
 * AprilTag to the spot directly under the Limelight's lens.
 *
 * How to use it:
 *   1. The Driver Station tells you how far away to put the robot (24 in, then 36 in, ...).
 *      Use a tape measure, then fine-tune with the "Camera measures" line, which says
 *      "move CLOSER" / "move BACK" / "OK".
 *   2. The motor spins at a test RPM. When the Driver Station says READY, feed ONE ball through.
 *   3. Tell it how the shot went:
 *        A = went in      -> that RPM is saved for this distance, and it moves to the next distance
 *        B = too much     -> it lowers the RPM for the next shot
 *        X = too little   -> it raises the RPM for the next shot
 *      Repeat 2 and 3 until a shot goes in.
 *
 *   It changes the RPM by STEP_RPM until it has seen a "too much" and a "too little", then it
 *   tries halfway between them, so it closes in on the right speed quickly. Each new distance
 *   starts with a guess that follows the trend of the RPMs that worked so far.
 *
 *   Other buttons (gamepad1):
 *        Y          = skip this distance (nothing saved for it)
 *        Dpad left  = undo the last button press (if you pressed the wrong one)
 *        Dpad down  = motor OFF,  Dpad up = motor ON
 *
 * Every time a shot goes in, the whole table is saved on the Robot Controller in
 * /sdcard/FIRST/settings/shooter_rpm_table.csv (see ShooterRpmTable). LimelightTagLockAutoRpm
 * loads that file when it starts, so you do not need to change any code. The first point you
 * save replaces the table that was there before (undoing it puts the old table back).
 *
 * The distance saved with each point is what the Limelight measured (not the tape measure),
 * because that is what LimelightTagLockAutoRpm will see during a match. If the camera cannot
 * see the tag when you press A, the asked-for distance is saved instead.
 *
 * The camera part (Limelight tag lock on the servo) is the same as LimelightTagLockAutoRpm.
 *
 * Robot Configuration:
 *   - Limelight 3A (plugged into USB, listed with the webcams)        name: "limelight"
 *   - Standard position servo (not continuous rotation) that turns
 *     the Limelight, on a servo port, type Servo                      name: "camServo"
 *   - goBILDA 6000 RPM motor, on a motor port                         name: "spinMotor"
 *
 * Limelight pipeline: an AprilTag pipeline (family 36h11) with 3D results turned on, and the
 * tag size set to the size of the printed tags. If the Driver Station says "no 3D data", check
 * the pipeline's 3D settings in the Limelight web interface.
 */
@TeleOp(name = "Shooter Distance Tuner", group = "Test")
public class ShooterDistanceTuner extends LinearOpMode {

    // =========================
    // Hardware names (must match the Robot Configuration)
    // =========================
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String CAM_SERVO_NAME = "camServo";
    private static final String MOTOR_NAME     = "spinMotor";

    // =========================
    // PART A: AprilTag
    // =========================
    private static final int PIPELINE_INDEX = 0;              // Limelight pipeline set up for AprilTags
    private static final int[] TARGET_TAG_IDS = { 34, 35, 36, 37 };   // the tags this program looks for

    // A result older than this means the Limelight has stopped sending data
    private static final long MAX_RESULT_AGE_MS = 250;

    // =========================
    // PART A: Camera servo THESE NEED TO BE MODIFIED SO YOU DONT BREAK THE WHOLE THING
    // =========================
    private static final double SERVO_MIN_POS   = 0.0;   // ends of the sweep
    private static final double SERVO_MAX_POS   = 1.0;
    private static final double SERVO_START_POS = 0.5;   // where the servo goes when you press Init

    // Degrees the camera turns when the servo goes from position 0.0 to 1.0.
    // 270 fits a goBILDA 300-degree servo on the SDK's default 600-2400 us range.
    // It turns the speeds below (in degrees) into servo movement, so close is good enough.
    private static final double SERVO_RANGE_DEG = 270.0;

    // true: a higher servo position turns the camera to the right (seen from behind the camera)
    private static final boolean HIGHER_POSITION_TURNS_RIGHT = true;

    // =========================
    // PART A, Section 1: search
    // =========================
    private static final double SEARCH_SPEED_DEG_PER_SEC = 30.0;   // sweep speed. Slower = less blur

    // =========================
    // PART A, Sections 2 and 3: align and stay locked
    // =========================
    // How hard the servo chases the tag: turn speed (deg/sec) = TRACK_GAIN * tx (deg).
    // Higher follows a moving tag more closely. Too high and the camera swings back and forth
    // across the tag. A slow Limelight pipeline (high resolution, low fps) needs a lower gain.
    private static final double TRACK_GAIN = 5.0;

    // tx this close to zero counts as zero. A servo cannot hold exactly 0.00 degrees.
    // If the servo twitches while locked, or Section 2 never hands over to Section 3, raise this.
    private static final double ON_TARGET_DEG = 0.5;

    // Section 2 hands over to Section 3 once tx has stayed at zero for this long
    private static final double LOCK_CONFIRM_SEC = 0.25;

    // Tag out of sight this long: give up and go back to Section 1. Shorter gaps (blur,
    // something passing in front of the tag) just make the servo wait where it is.
    private static final double LOST_TIMEOUT_SEC = 0.5;

    // Longest time step used for one loop, so one slow loop cannot throw the servo a long way
    private static final double MAX_LOOP_SEC = 0.1;

    private static final int NO_TAG = -1;

    // =========================
    // PART B: Motor
    // =========================
    // goBILDA 6000 RPM Yellow Jacket: 28 encoder ticks per output shaft revolution (1:1 gearbox)
    private static final double TICKS_PER_REV = 28.0;

    private static final double MAX_RPM      = 6000.0;
    private static final double MIN_TEST_RPM = 500.0;   // the tuner never tries slower than this

    // Flip to REVERSE if the motor spins the wrong way
    private static final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // =========================
    // PART B: Tuning
    // =========================
    // The floor distances (inches) the tuner asks for, in order, closest first.
    // Add, remove or change these to match where you will shoot from.
    private static final double[] TEST_DISTANCES_IN = { 24, 36, 48, 60, 72, 96, 120 };

    // RPM of the first test shot at the first distance. Later distances start with a guess
    // from the RPMs that worked so far.
    private static final double FIRST_TEST_RPM = 2500.0;

    // How much the RPM changes after "too much" / "too little", until there has been at least
    // one of each. After that it tries halfway between the closest "too much" and "too little".
    private static final double STEP_RPM = 200.0;

    // Test RPMs are rounded to this
    private static final double RPM_ROUNDING = 10.0;

    // READY: the motor has stayed within READY_TOL_RPM of the test RPM for READY_HOLD_SEC.
    // The count starts over every time the test RPM changes, so the motor has time to settle.
    private static final double READY_TOL_RPM  = 100.0;
    private static final double READY_HOLD_SEC = 0.5;

    // "OK" on the Driver Station when the measured distance is this close to the asked-for one
    private static final double POSITION_TOL_IN = 2.0;

    // Smooths out jitter in the distance reading. 1.0 = no smoothing, smaller = smoother.
    private static final double DISTANCE_SMOOTHING = 0.3;

    // Readings outside this range are treated as bad data and ignored
    private static final double MIN_VALID_DISTANCE_IN = 2.0;
    private static final double MAX_VALID_DISTANCE_IN = 240.0;

    // A distance reading older than this is too old to save with a point
    private static final double DISTANCE_FRESH_SEC = 1.0;

    private static final double UNKNOWN = -1.0;

    // =========================
    // Hardware
    // =========================
    private Limelight3A limelight;
    private Servo camServo;
    private DcMotorEx motor;

    // =========================
    // PART A state
    // =========================
    private enum Section { SEARCH, ALIGN, LOCKED }

    private Section section = Section.SEARCH;
    private double servoPosition = SERVO_START_POS;   // where the servo is being told to go
    private int searchDirection = +1;                 // +1: servo position going up, -1: going down
    private int followedTagId = NO_TAG;               // the tag (34-37) Sections 2 and 3 follow

    // What the camera sees on this loop
    private boolean limelightHasData;
    private LLResultTypes.FiducialResult tag;         // the target tag, or null if it is not in view
    private double lastTx;                            // tx the last time the tag was in view

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime sinceTagSeen = new ElapsedTime();
    private final ElapsedTime onTargetTimer = new ElapsedTime();   // how long tx has stayed at zero

    // =========================
    // PART B state
    // =========================
    private int distanceIndex = 0;               // which of TEST_DISTANCES_IN we are on
    private double testRpm = FIRST_TEST_RPM;     // RPM the motor is spinning at for the next shot
    private double tooLittleRpm = UNKNOWN;       // highest RPM that came up short at this distance
    private double tooMuchRpm = UNKNOWN;         // lowest RPM that went too far at this distance
    private int shotsHere = 0;                   // shots taken at this distance
    private boolean motorOn = true;
    private boolean finished = false;

    // Points saved so far (same order in all three lists)
    private final List<Double> savedDistanceIn = new ArrayList<>();
    private final List<Double> savedRpm = new ArrayList<>();
    private final List<Boolean> savedFromTape = new ArrayList<>();   // true: camera had no distance

    private String lastAction = "none yet";
    private String saveStatus = "nothing saved yet";
    private String fileBeforeTuning = null;      // the file's text before this run (null: no file)

    private Snapshot undo = null;                // state before the last button press

    // Distance to the tag
    private boolean haveDistance = false;
    private double smoothedDistanceIn = 0.0;
    private double rawDistanceIn = 0.0;          // this loop's reading (0 if none)
    private boolean noDataFromTag = false;       // tag in view but the Limelight sent no 3D data
    private double cameraBelowTagIn = 0.0;
    private final ElapsedTime sinceDistance = new ElapsedTime();

    private final ElapsedTime readyTimer = new ElapsedTime();
    private boolean prevA, prevB, prevX, prevY, prevLeft;

    /** Everything one button press can change, so Dpad left can put it back. */
    private static class Snapshot {
        final int distanceIndex;
        final double testRpm, tooLittleRpm, tooMuchRpm;
        final int shotsHere, savedCount;
        final boolean finished;

        Snapshot(int distanceIndex, double testRpm, double tooLittleRpm, double tooMuchRpm,
                 int shotsHere, int savedCount, boolean finished) {
            this.distanceIndex = distanceIndex;
            this.testRpm = testRpm;
            this.tooLittleRpm = tooLittleRpm;
            this.tooMuchRpm = tooMuchRpm;
            this.shotsHere = shotsHere;
            this.savedCount = savedCount;
            this.finished = finished;
        }
    }

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        camServo = hardwareMap.get(Servo.class, CAM_SERVO_NAME);
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        // Motor setup. FLOAT so it coasts to a stop instead of slamming the brakes at high RPM.
        motor.setDirection(MOTOR_DIRECTION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0.0);

        // A servo cannot report where it is, so send it to a known position to start from
        camServo.setPosition(servoPosition);

        // Remember the saved table from before, so undoing the first new point can put it back
        File tableFile = ShooterRpmTable.file();
        if (tableFile.exists()) fileBeforeTuning = ReadWriteFile.readFile(tableFile);
        ShooterRpmTable oldTable = ShooterRpmTable.loadOrNull();

        // Send telemetry often so the Driver Station keeps up with the camera
        telemetry.setMsTransmissionInterval(11);

        // The Limelight takes a while to boot, so keep asking until it switches pipelines
        while (opModeInInit() && !limelight.pipelineSwitch(PIPELINE_INDEX)) {
            telemetry.addData("Status", "Waiting for the Limelight to respond");
            telemetry.update();
            sleep(500);
        }

        // Without start(), getLatestResult() always returns null
        limelight.start();

        telemetry.addData("Status", "Ready. Press Play to start tuning");
        telemetry.addLine("You will shoot from " + TEST_DISTANCES_IN.length + " distances: " + distancesText());
        telemetry.addLine("A = went in | B = too much | X = too little");
        telemetry.addLine("Y = skip | Dpad left = undo | Dpad down/up = motor off/on");
        if (oldTable != null) {
            telemetry.addLine("A saved table with " + oldTable.size() + " point(s) is on the Robot Controller."
                    + " The first point you save replaces it.");
        }
        telemetry.update();

        waitForStart();
        loopTimer.reset();
        readyTimer.reset();

        // Don't count a button that was already held when Play was pressed
        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevLeft = gamepad1.dpad_left;

        while (opModeIsActive()) {
            // Seconds since the last loop, so servo speeds do not depend on how fast the loop runs
            double dt = Math.min(loopTimer.seconds(), MAX_LOOP_SEC);
            loopTimer.reset();

            // PART A: Limelight tag lock
            lookForTag();

            switch (section) {
                case SEARCH: section1Search(dt); break;
                case ALIGN:  section2Align(dt);  break;
                case LOCKED: section3Locked(dt); break;
            }

            camServo.setPosition(servoPosition);

            // PART B: Tuning
            updateTuner();

            showTelemetry();
        }

        motor.setPower(0.0);
        limelight.stop();
    }

    // =====================================================================================
    // PART B - TUNING
    // =====================================================================================
    private void updateTuner() {
        measureDistance();

        // Edge detection: only react on the moment a button goes from released -> pressed
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y, left = gamepad1.dpad_left;
        boolean aPressed = a && !prevA;
        boolean bPressed = b && !prevB;
        boolean xPressed = x && !prevX;
        boolean yPressed = y && !prevY;
        boolean leftPressed = left && !prevLeft;
        prevA = a;
        prevB = b;
        prevX = x;
        prevY = y;
        prevLeft = left;

        if (gamepad1.dpad_down) motorOn = false;
        if (gamepad1.dpad_up) motorOn = true;

        if (leftPressed) {
            undoLastPress();
        } else if (!finished && (aPressed || bPressed || xPressed || yPressed)) {
            if (yPressed) {
                remember();
                lastAction = String.format("SKIPPED %.0f in", TEST_DISTANCES_IN[distanceIndex]);
                nextDistance();
            } else if (!motorOn) {
                lastAction = "motor is OFF - press Dpad up, then shoot";
            } else if (aPressed) {
                remember();
                wentIn();
            } else if (bPressed) {
                remember();
                tooMuch();
            } else {
                remember();
                tooLittle();
            }
        }

        if (finished || !motorOn) {
            // Cut power (coast) rather than commanding 0 velocity, which would actively brake
            motor.setPower(0.0);
            readyTimer.reset();
        } else {
            motor.setVelocity(rpmToTicksPerSec(testRpm));
            if (Math.abs(actualRpm() - testRpm) > READY_TOL_RPM) readyTimer.reset();
        }
    }

    /** A: the shot went in. Save this RPM for this distance and move on. */
    private void wentIn() {
        boolean cameraHasDistance = haveDistance && sinceDistance.seconds() <= DISTANCE_FRESH_SEC;
        double distance = cameraHasDistance ? smoothedDistanceIn : TEST_DISTANCES_IN[distanceIndex];

        savedDistanceIn.add(distance);
        savedRpm.add(testRpm);
        savedFromTape.add(!cameraHasDistance);
        saveToFile();

        lastAction = String.format("WENT IN at %.0f RPM - saved %.1f in = %.0f RPM%s",
                testRpm, distance, testRpm, cameraHasDistance ? "" : " (camera had no distance)");
        nextDistance();
    }

    /** B: the shot went too far. Lower the RPM. */
    private void tooMuch() {
        shotsHere++;
        double rpm = testRpm;
        if (tooMuchRpm == UNKNOWN || rpm < tooMuchRpm) tooMuchRpm = rpm;
        // Shots vary. If an earlier "too little" was at this RPM or higher, forget it.
        if (tooLittleRpm != UNKNOWN && tooLittleRpm >= tooMuchRpm) tooLittleRpm = UNKNOWN;

        double next = (tooLittleRpm != UNKNOWN) ? (tooLittleRpm + tooMuchRpm) / 2.0 : rpm - STEP_RPM;
        next = roundRpm(next);
        if (next >= rpm) next = rpm - RPM_ROUNDING;   // always go down after "too much"
        testRpm = Range.clip(next, MIN_TEST_RPM, MAX_RPM);
        readyTimer.reset();   // new RPM: wait for the motor to settle before READY

        lastAction = String.format("TOO MUCH at %.0f - next try %.0f", rpm, testRpm);
        if (rpm <= MIN_TEST_RPM) lastAction += " (already at the lowest test RPM: move back or press Y)";
    }

    /** X: the shot came up short. Raise the RPM. */
    private void tooLittle() {
        shotsHere++;
        double rpm = testRpm;
        if (tooLittleRpm == UNKNOWN || rpm > tooLittleRpm) tooLittleRpm = rpm;
        // Shots vary. If an earlier "too much" was at this RPM or lower, forget it.
        if (tooMuchRpm != UNKNOWN && tooMuchRpm <= tooLittleRpm) tooMuchRpm = UNKNOWN;

        double next = (tooMuchRpm != UNKNOWN) ? (tooLittleRpm + tooMuchRpm) / 2.0 : rpm + STEP_RPM;
        next = roundRpm(next);
        if (next <= rpm) next = rpm + RPM_ROUNDING;   // always go up after "too little"
        testRpm = Range.clip(next, MIN_TEST_RPM, MAX_RPM);
        readyTimer.reset();   // new RPM: wait for the motor to settle before READY

        lastAction = String.format("TOO LITTLE at %.0f - next try %.0f", rpm, testRpm);
        if (rpm >= MAX_RPM) lastAction += " (already at max RPM: move closer or press Y)";
    }

    private void nextDistance() {
        distanceIndex++;
        tooLittleRpm = UNKNOWN;
        tooMuchRpm = UNKNOWN;
        shotsHere = 0;
        readyTimer.reset();
        if (distanceIndex >= TEST_DISTANCES_IN.length) {
            distanceIndex = TEST_DISTANCES_IN.length - 1;   // keep the index valid for telemetry
            finished = true;
            return;
        }
        testRpm = firstGuessFor(TEST_DISTANCES_IN[distanceIndex]);
    }

    /**
     * First RPM to try at a new distance. With two or more saved points, it follows the line
     * through the last two (farther away = more RPM). With one, it starts at that point's RPM.
     * With none, it keeps the current test RPM.
     */
    private double firstGuessFor(double askedIn) {
        int n = savedDistanceIn.size();
        if (n == 0) return testRpm;

        double lastIn = savedDistanceIn.get(n - 1);
        double lastRpm = savedRpm.get(n - 1);
        if (n == 1) return lastRpm;

        double prevIn = savedDistanceIn.get(n - 2);
        double prevRpm = savedRpm.get(n - 2);
        if (lastIn - prevIn < 1.0) return lastRpm;   // too close together to show a trend

        double rpmPerInch = (lastRpm - prevRpm) / (lastIn - prevIn);
        if (rpmPerInch <= 0) return lastRpm;         // no upward trend: just start at the last RPM

        return Range.clip(roundRpm(lastRpm + rpmPerInch * (askedIn - lastIn)), MIN_TEST_RPM, MAX_RPM);
    }

    private void remember() {
        undo = new Snapshot(distanceIndex, testRpm, tooLittleRpm, tooMuchRpm,
                shotsHere, savedDistanceIn.size(), finished);
    }

    /** Dpad left: put everything back the way it was before the last button press. */
    private void undoLastPress() {
        if (undo == null) {
            lastAction = "nothing to undo";
            return;
        }
        boolean removedPoint = savedDistanceIn.size() > undo.savedCount;
        while (savedDistanceIn.size() > undo.savedCount) {
            int lastIndex = savedDistanceIn.size() - 1;
            savedDistanceIn.remove(lastIndex);
            savedRpm.remove(lastIndex);
            savedFromTape.remove(lastIndex);
        }

        distanceIndex = undo.distanceIndex;
        testRpm = undo.testRpm;
        tooLittleRpm = undo.tooLittleRpm;
        tooMuchRpm = undo.tooMuchRpm;
        shotsHere = undo.shotsHere;
        finished = undo.finished;
        undo = null;   // one step of undo only

        if (removedPoint) saveToFile();
        lastAction = "UNDID the last button press" + (removedPoint ? " (removed the saved point)" : "");
        readyTimer.reset();
    }

    /** Writes all saved points to the file. With no points, puts back the file from before. */
    private void saveToFile() {
        try {
            if (savedDistanceIn.isEmpty()) {
                File f = ShooterRpmTable.file();
                if (fileBeforeTuning != null) {
                    ReadWriteFile.writeFileOrThrow(f, fileBeforeTuning);
                } else if (f.exists() && !f.delete()) {
                    throw new IOException("could not delete " + f.getPath());
                }
                saveStatus = "no points saved this run (file put back the way it was)";
                return;
            }
            new ShooterRpmTable(toArray(savedDistanceIn), toArray(savedRpm)).save();
            saveStatus = String.format("%d point(s) saved to %s", savedDistanceIn.size(), ShooterRpmTable.file().getPath());
        } catch (IOException | RuntimeException e) {
            saveStatus = "SAVE FAILED: " + e.getMessage();
        }
    }

    /**
     * Reads the floor distance to the tag being tracked into rawDistanceIn (0 if there is no
     * good reading this loop), and updates the smoothed distance that gets saved.
     */
    private void measureDistance() {
        rawDistanceIn = 0.0;
        noDataFromTag = false;
        if (tag == null) return;

        double distance = ShooterRpmTable.floorDistanceInches(tag);
        if (distance < MIN_VALID_DISTANCE_IN || distance > MAX_VALID_DISTANCE_IN) {
            noDataFromTag = true;
            return;
        }

        rawDistanceIn = distance;
        cameraBelowTagIn = ShooterRpmTable.cameraBelowTagInches(tag);
        if (!haveDistance || sinceDistance.seconds() > DISTANCE_FRESH_SEC) {
            smoothedDistanceIn = distance;   // first reading, or the old one is stale: start over here
            haveDistance = true;
        } else {
            smoothedDistanceIn += DISTANCE_SMOOTHING * (distance - smoothedDistanceIn);
        }
        sinceDistance.reset();
    }

    private double actualRpm() {
        return ticksPerSecToRpm(motor.getVelocity());
    }

    private static double roundRpm(double rpm) {
        return Math.round(rpm / RPM_ROUNDING) * RPM_ROUNDING;
    }

    private static double[] toArray(List<Double> list) {
        double[] out = new double[list.size()];
        for (int i = 0; i < out.length; i++) out[i] = list.get(i);
        return out;
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }

    // =====================================================================================
    // PART A, SECTION 1 - SEARCH
    // The servo slowly sweeps the camera until it sees AprilTag 34, 35, 36 or 37.
    // =====================================================================================
    private void section1Search(double dt) {
        if (tag != null) {
            // Found one. Remember which, so Sections 2 and 3 stay on this tag.
            followedTagId = tag.getFiducialId();
            onTargetTimer.reset();
            section = Section.ALIGN;
            return;
        }

        // No camera data: wait here instead of sweeping past a tag the camera cannot report
        if (!limelightHasData) return;

        servoPosition += searchDirection * SEARCH_SPEED_DEG_PER_SEC * dt / SERVO_RANGE_DEG;

        // Turn around at each end of the sweep
        if (servoPosition >= SERVO_MAX_POS) {
            servoPosition = SERVO_MAX_POS;
            searchDirection = -1;
        } else if (servoPosition <= SERVO_MIN_POS) {
            servoPosition = SERVO_MIN_POS;
            searchDirection = +1;
        }
    }

    // =====================================================================================
    // PART A, SECTION 2 - ALIGN
    // The servo constantly adjusts its position to make tx = zero degrees.
    // =====================================================================================
    private void section2Align(double dt) {
        if (tagIsLost()) {
            goBackToSearch();
            return;
        }
        if (tag == null) {
            // Out of sight for a moment: hold still, and start the on-target count over
            onTargetTimer.reset();
            return;
        }

        double tx = tag.getTargetXDegrees();
        turnToward(tx, dt);

        // Locked once tx has stayed at zero for LOCK_CONFIRM_SEC without a break
        if (Math.abs(tx) > ON_TARGET_DEG) onTargetTimer.reset();
        if (onTargetTimer.seconds() >= LOCK_CONFIRM_SEC) section = Section.LOCKED;
    }

    // =====================================================================================
    // PART A, SECTION 3 - LOCKED
    // The target is locked. If any movement occurs, the servo keeps the camera on target.
    // =====================================================================================
    private void section3Locked(double dt) {
        if (tagIsLost()) {
            goBackToSearch();
            return;
        }
        if (tag == null) return;   // out of sight for a moment: hold still

        // tx only leaves zero when the tag or the robot moves. Turn the camera to follow.
        double tx = tag.getTargetXDegrees();
        if (Math.abs(tx) > ON_TARGET_DEG) turnToward(tx, dt);
    }

    // =====================================================================================
    // PART A - shared by the sections
    // =====================================================================================

    /** Reads the newest Limelight result and picks out the tag the current section wants. */
    private void lookForTag() {
        LLResult result = limelight.getLatestResult();
        limelightHasData = result != null && result.getStaleness() <= MAX_RESULT_AGE_MS;

        tag = limelightHasData ? findTag(result.getFiducialResults()) : null;
        if (tag != null) {
            lastTx = tag.getTargetXDegrees();
            sinceTagSeen.reset();
        }
    }

    /** The wanted tag, or null. If more than one is in view, the biggest (closest) one. */
    private LLResultTypes.FiducialResult findTag(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null) return null;

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult candidate : tags) {
            if (!isWanted(candidate.getFiducialId())) continue;
            if (best == null || candidate.getTargetArea() > best.getTargetArea()) best = candidate;
        }
        return best;
    }

    /**
     * Section 1 wants tag 34, 35, 36 or 37. Sections 2 and 3 want only the tag they are following,
     * so the camera does not jump between tags when more than one is in view.
     */
    private boolean isWanted(int tagId) {
        if (section != Section.SEARCH) return tagId == followedTagId;

        for (int targetId : TARGET_TAG_IDS) {
            if (tagId == targetId) return true;
        }
        return false;
    }

    /**
     * Turns the camera toward the tag. The turn speed is proportional to tx, so the camera
     * moves quickly when the tag is far off center and eases to a stop as tx reaches zero.
     */
    private void turnToward(double txDegrees, double dt) {
        double degreesRight = TRACK_GAIN * txDegrees * dt;
        double positionChange = degreesRight / SERVO_RANGE_DEG;
        if (!HIGHER_POSITION_TURNS_RIGHT) positionChange = -positionChange;

        servoPosition = Range.clip(servoPosition + positionChange, SERVO_MIN_POS, SERVO_MAX_POS);
    }

    private boolean tagIsLost() {
        return tag == null && sinceTagSeen.seconds() > LOST_TIMEOUT_SEC;
    }

    /** Back to Section 1, sweeping first toward the side where the tag was last seen. */
    private void goBackToSearch() {
        boolean lastSeenOnRight = lastTx > 0;
        searchDirection = (lastSeenOnRight == HIGHER_POSITION_TURNS_RIGHT) ? +1 : -1;
        followedTagId = NO_TAG;
        section = Section.SEARCH;
    }

    // =====================================================================================
    // Telemetry
    // =====================================================================================
    private void showTelemetry() {
        if (finished) {
            showFinished();
            return;
        }

        double askedIn = TEST_DISTANCES_IN[distanceIndex];
        telemetry.addData("SHOOTER TUNER", "distance %d of %d", distanceIndex + 1, TEST_DISTANCES_IN.length);
        telemetry.addLine(String.format("1) Put the Limelight lens %.0f in from the floor spot under the tag", askedIn));
        telemetry.addData("   Camera measures", distanceGuide(askedIn));
        telemetry.addLine("2) Wait for READY, then feed ONE ball");
        telemetry.addData("   Test RPM", "%.0f  (actual %.0f)  %s", testRpm, actualRpm(), readyText());
        telemetry.addLine("3) A = went in | B = too much | X = too little");
        telemetry.addData("   Last", lastAction);
        telemetry.addData("   Shots here", "%d  (%s)", shotsHere, boundsText());
        telemetry.addLine("Y = skip | Dpad left = undo | Dpad down/up = motor off/on");

        telemetry.addLine();
        telemetry.addData("Saved", saveStatus);
        addSavedPoints();

        // PART A: Limelight
        telemetry.addLine();
        switch (section) {
            case SEARCH:
                telemetry.addData("Section", "1 - SEARCHING for AprilTag %s", targetIdsText());
                break;
            case ALIGN:
                telemetry.addData("Section", "2 - ALIGNING on AprilTag %d", followedTagId);
                break;
            case LOCKED:
                telemetry.addData("Section", "3 - LOCKED on AprilTag %d", followedTagId);
                break;
        }

        if (!limelightHasData) {
            telemetry.addData("tx", "waiting for Limelight data");
        } else if (tag == null) {
            telemetry.addData("tx", "tag not in view");
        } else {
            telemetry.addData("tx", txText(tag.getTargetXDegrees()));
        }

        boolean atEndOfTravel = servoPosition <= SERVO_MIN_POS || servoPosition >= SERVO_MAX_POS;
        telemetry.addData("Servo position", atEndOfTravel ? "%.3f (end of travel)" : "%.3f", servoPosition);

        if (haveDistance) {
            // Sanity check: should match the real height difference between the tag center and the lens
            telemetry.addData("Height check", "lens is %.1f in %s the tag center",
                    Math.abs(cameraBelowTagIn), cameraBelowTagIn >= 0 ? "below" : "above");
        }

        telemetry.update();
    }

    private void showFinished() {
        telemetry.addData("SHOOTER TUNER", "ALL DONE - motor off");
        telemetry.addData("Saved", saveStatus);
        addSavedPoints();
        telemetry.addLine();
        if (!savedDistanceIn.isEmpty()) {
            telemetry.addLine("LimelightTagLockAutoRpm will use this table the next time it starts.");
            telemetry.addLine("To put it in the code instead, copy these into LimelightTagLockAutoRpm:");
            telemetry.addLine(new ShooterRpmTable(toArray(savedDistanceIn), toArray(savedRpm)).toJavaArrays());
        }
        telemetry.addLine("Dpad left = undo the last button press");
        telemetry.update();
    }

    private void addSavedPoints() {
        for (int i = 0; i < savedDistanceIn.size(); i++) {
            telemetry.addLine(String.format("   %.1f in -> %.0f RPM%s", savedDistanceIn.get(i), savedRpm.get(i),
                    savedFromTape.get(i) ? "  (asked-for distance, camera had none)" : ""));
        }
    }

    /** How far off the robot is from the asked-for distance, and which way to move. */
    private String distanceGuide(double askedIn) {
        if (rawDistanceIn > 0.0) {
            double off = smoothedDistanceIn - askedIn;
            if (Math.abs(off) <= POSITION_TOL_IN) return String.format("%.1f in - OK", smoothedDistanceIn);
            return String.format("%.1f in - move %s %.1f in", smoothedDistanceIn, off > 0 ? "CLOSER" : "BACK", Math.abs(off));
        }
        if (noDataFromTag) return "tag in view but no 3D data (check the Limelight pipeline's 3D settings)";
        return "tag not in view";
    }

    private String readyText() {
        if (!motorOn) return "MOTOR OFF (Dpad up to turn on)";
        if (readyTimer.seconds() >= READY_HOLD_SEC) return "READY - feed the ball";
        return "spinning up...";
    }

    private String boundsText() {
        String little = tooLittleRpm == UNKNOWN ? "none" : String.format("%.0f", tooLittleRpm);
        String much = tooMuchRpm == UNKNOWN ? "none" : String.format("%.0f", tooMuchRpm);
        return "too little up to " + little + ", too much from " + much;
    }

    /** "24, 36, 48 ... in" */
    private static String distancesText() {
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < TEST_DISTANCES_IN.length; i++) {
            if (i > 0) text.append(", ");
            text.append(String.format("%.0f", TEST_DISTANCES_IN[i]));
        }
        return text.append(" in").toString();
    }

    private static String txText(double txDegrees) {
        if (Math.abs(txDegrees) < 0.05) return "0.0 deg (centered)";
        return String.format("%.1f deg %s of center", Math.abs(txDegrees), txDegrees > 0 ? "right" : "left");
    }

    /** "34, 35, 36 or 37" */
    private static String targetIdsText() {
        StringBuilder ids = new StringBuilder();
        for (int i = 0; i < TARGET_TAG_IDS.length; i++) {
            if (i > 0) ids.append(i == TARGET_TAG_IDS.length - 1 ? " or " : ", ");
            ids.append(TARGET_TAG_IDS[i]);
        }
        return ids.toString();
    }
}
