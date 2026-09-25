// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBTeleOp_ShooterRPMTuner.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.oysterbay.base.RisingEdge;
import org.firstinspires.ftc.teamcode.oysterbay.base.RobotStructure;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Intake;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.ShotSequencer;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Tipper;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Trap;

/**
 * Shooter RPM Fine Tuner (setVelocity version that matches your current shooter sign logic)
 *
 * Goal: park in one place, repeatedly shoot, and tune RPM until arc is perfect.
 *
 * Controls (gamepad1):
 *  - Drive: normal RobotStructure drive
 *  - Intake: A (hold)
 *  - Trap:   B (hold)
 *
 * Shooter:
 *  - Right trigger: slight press spins up, full press fires (same behavior as driver)
 *
 * RPM adjust (4 buttons total):
 *  - Right bumper: +150 RPM
 *  - Left bumper:  -150 RPM
 *  - Dpad up:      +10 RPM
 *  - Dpad down:    -10 RPM
 *
 * Optional tuning helpers:
 *  - X: toggle require-at-speed (lets you choose "always fire" vs "fire only at speed")
 *  - Dpad left/right: adjust at-speed tolerance by +/-10 (how tight "at speed" is)
 *  - START: reset RPM to default
 */
@TeleOp(name = "OB Shooter RPM Tuner (Velocity)", group = "OB")
public class OBTeleOp_ShooterRPMTuner extends OpMode {

    // RPM tuning
    private static final double DEFAULT_RPM = 4500.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;

    private static final double COARSE_STEP_RPM = 150.0;
    private static final double FINE_STEP_RPM   = 10.0;

    private static final double TOL_STEP_RPM = 10.0;
    private static final double MIN_TOL_RPM  = 20.0;
    private static final double MAX_TOL_RPM  = 400.0;

    private static final double DRIVE_SPEED = 0.55;

    // Trap powers used while tuning (stronger left side than the driver TeleOp)
    private static final double TRAP_LEFT_POWER  = -1.0;
    private static final double TRAP_RIGHT_POWER = 0.2;

    private final RobotStructure robot = new RobotStructure();

    private Shooter shooter;
    private Intake intake;
    private Tipper tipper;
    private Trap trap;
    private ShotSequencer sequencer;

    private double targetRpm = DEFAULT_RPM;

    private final RisingEdge rbEdge = new RisingEdge();
    private final RisingEdge lbEdge = new RisingEdge();
    private final RisingEdge upEdge = new RisingEdge();
    private final RisingEdge downEdge = new RisingEdge();
    private final RisingEdge leftEdge = new RisingEdge();
    private final RisingEdge rightEdge = new RisingEdge();
    private final RisingEdge xEdge = new RisingEdge();
    private final RisingEdge startEdge = new RisingEdge();

    @Override
    public void init() {
        robot.init(hardwareMap);

        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        tipper  = new Tipper(hardwareMap);
        trap    = new Trap(hardwareMap);

        sequencer = new ShotSequencer(shooter, tipper,
                new ShotSequencer.Channel("TUNE",
                        () -> Range.clip(gamepad1.right_trigger, 0.0, 1.0),
                        () -> targetRpm));

        telemetry.addLine("OB Shooter RPM Tuner (Velocity) ready");
        telemetry.addLine("RB/LB: +/-150 RPM | DpadUp/Down: +/-10 RPM");
        telemetry.addLine("RT: spin | Full RT: fire (same feel as driver)");
        telemetry.addLine("X: toggle require-at-speed | Dpad L/R: tol +/-10 | START: reset RPM");
        telemetry.update();
    }

    @Override
    public void loop() {
        // One bulk read per hub this loop; all encoder/velocity reads below come from the cache
        robot.clearBulkCache();

        robot.driveFromGamepad(gamepad1, true, DRIVE_SPEED);
        intake.feedWhile(gamepad1.a);
        runTrap(gamepad1.b);

        adjustTargetRpm();
        adjustFiringGate();

        sequencer.update();
        addTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        trap.stop();
        tipper.rest();
        robot.setDriverPowerZERO();
    }

    // =========================
    // Procedures
    // =========================
    private void runTrap(boolean held) {
        if (held) trap.set(TRAP_LEFT_POWER, TRAP_RIGHT_POWER);
        else trap.stop();
    }

    /** Bumpers = coarse steps, D-pad up/down = fine steps, START = reset. */
    private void adjustTargetRpm() {
        if (rbEdge.update(gamepad1.right_bumper)) targetRpm += COARSE_STEP_RPM;
        if (lbEdge.update(gamepad1.left_bumper))  targetRpm -= COARSE_STEP_RPM;
        if (upEdge.update(gamepad1.dpad_up))      targetRpm += FINE_STEP_RPM;
        if (downEdge.update(gamepad1.dpad_down))  targetRpm -= FINE_STEP_RPM;
        targetRpm = Range.clip(targetRpm, MIN_RPM, MAX_RPM);

        if (startEdge.update(gamepad1.start)) targetRpm = DEFAULT_RPM;
    }

    /** D-pad left/right = at-speed tolerance, X = toggle whether firing waits for speed. */
    private void adjustFiringGate() {
        double tol = sequencer.getAtSpeedTolRpm();
        if (rightEdge.update(gamepad1.dpad_right)) tol += TOL_STEP_RPM;
        if (leftEdge.update(gamepad1.dpad_left))   tol -= TOL_STEP_RPM;
        sequencer.setAtSpeedTolRpm(Range.clip(tol, MIN_TOL_RPM, MAX_TOL_RPM));

        if (xEdge.update(gamepad1.x)) {
            sequencer.setRequireAtSpeedToFire(!sequencer.isRequireAtSpeedToFire());
        }
    }

    private void addTelemetry() {
        double lVel = shooter.leftRawVelocity();
        double rVel = shooter.rightRawVelocity();
        double lRpm = shooter.leftRpm();
        double rRpm = shooter.rightRpm();
        double tol = sequencer.getAtSpeedTolRpm();

        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("L RPM (shooting+)", "%.0f", lRpm);
        telemetry.addData("R RPM (shooting+)", "%.0f", rRpm);
        telemetry.addData("Tol RPM", "%.0f", tol);
        telemetry.addData("At speed?", Shooter.bothWithin(lRpm, rRpm, targetRpm, tol));
        telemetry.addData("Require at speed?", sequencer.isRequireAtSpeedToFire());
        telemetry.addData("Trigger", "%.2f", Range.clip(gamepad1.right_trigger, 0.0, 1.0));
        telemetry.addData("State", sequencer.state());
        telemetry.addData("Tipper", tipper.isFiring() ? "FIRING" : "REST");

        // Raw encoder signs for debugging
        telemetry.addData("Raw L pos", shooter.leftPosition());
        telemetry.addData("Raw R pos", shooter.rightPosition());
        telemetry.addData("Raw L vel (t/s)", "%.0f", lVel);
        telemetry.addData("Raw R vel (t/s)", "%.0f", rVel);

        telemetry.update();
    }
}
