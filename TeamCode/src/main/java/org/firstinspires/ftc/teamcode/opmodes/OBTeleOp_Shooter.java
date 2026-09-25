// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBTeleOp_Shooter.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.oysterbay.base.RobotStructure;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Holders;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Intake;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.ShotSequencer;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.TagVision;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Tipper;
import org.firstinspires.ftc.teamcode.oysterbay.subsystems.Trap;

/**
 * Driver TeleOp with a velocity-controlled shooter.
 *
 * Gamepad 1:
 *  - Left stick / right stick X: drive / turn (X held = speed boost)
 *  - A: intake, B: trap feed
 *  - D-pad: LEFT opens left holder, RIGHT opens right holder, UP opens both, DOWN closes both
 *  - Right trigger: long shot, left trigger: short shot (light pull spins up, full pull fires)
 *  - Y (held): reverse everything to clear a jam
 */
@TeleOp(name = "OB TeleOp + Shooter (Velocity)", group = "OB")
public class OBTeleOp_Shooter extends OpMode {

    // Long vs Short RPM
    private static final double LONG_TARGET_RPM  = 1950.0; // right trigger
    private static final double SHORT_TARGET_RPM = 1800.0; // left trigger

    private static final double CLEAR_JAM_RPM = -900.0;

    private static final double NORMAL_SPEED = 0.8;
    private static final double BOOST_SPEED  = 1.5;

    private final RobotStructure robot = new RobotStructure();

    private Shooter shooter;
    private Intake intake;
    private Trap trap;
    private Tipper tipper;
    private Holders holders;
    private TagVision vision;
    private ShotSequencer sequencer;

    @Override
    public void init() {
        robot.init(hardwareMap);

        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
        trap    = new Trap(hardwareMap);
        tipper  = new Tipper(hardwareMap);
        holders = Holders.forTeleOp(hardwareMap);

        // Camera for Driver Station live view
        vision = new TagVision(hardwareMap, null);

        // Right trigger has priority if both are pulled
        sequencer = new ShotSequencer(shooter, tipper,
                new ShotSequencer.Channel("LONG",  () -> trigger(gamepad1.right_trigger), () -> LONG_TARGET_RPM),
                new ShotSequencer.Channel("SHORT", () -> trigger(gamepad1.left_trigger),  () -> SHORT_TARGET_RPM));

        telemetry.addLine("OB TeleOp + Shooter (Velocity) ready");
        telemetry.addLine("Right trigger = long shot, Left trigger = short shot");
        telemetry.addData("Long RPM", "%.0f", LONG_TARGET_RPM);
        telemetry.addData("Short RPM", "%.0f", SHORT_TARGET_RPM);
        telemetry.update();
    }

    @Override
    public void loop() {
        // One bulk read per hub this loop; all encoder/velocity reads below come from the cache
        robot.clearBulkCache();

        drive();
        trap.feedWhile(gamepad1.b);
        holders.updateFromDpad(gamepad1);
        intake.feedWhile(gamepad1.a);

        if (gamepad1.y) {
            clearJam();
            telemetry.addData("OVERRIDE", "Y held (reverse clear)");
            telemetry.update();
            return;
        }

        sequencer.update();
        addTelemetry();
    }

    @Override
    public void stop() {
        shooter.stop();
        intake.stop();
        tipper.rest();
        trap.stop();
        robot.setDriverPowerZERO();
        vision.close();
    }

    // =========================
    // Procedures
    // =========================
    private void drive() {
        double speedMult = gamepad1.x ? BOOST_SPEED : NORMAL_SPEED;
        robot.driveFromGamepad(gamepad1, true, speedMult);
    }

    /** Run shooter, trap and intake backwards to push a stuck ball out. */
    private void clearJam() {
        shooter.setRpm(CLEAR_JAM_RPM);
        trap.reverse();
        intake.reverse();
    }

    private static double trigger(float value) {
        return Range.clip(value, 0.0, 1.0);
    }

    private void addTelemetry() {
        ShotSequencer.Channel active = sequencer.active();
        double target = sequencer.targetRpm();

        telemetry.addData("Camera", vision.cameraState());
        telemetry.addData("Tag detections", vision.detections().size());

        telemetry.addData("Mode", active == null ? "NONE" : active.name);
        telemetry.addData("Long Trigger", "%.3f", trigger(gamepad1.right_trigger));
        telemetry.addData("Short Trigger", "%.3f", trigger(gamepad1.left_trigger));

        telemetry.addData("Shooter L pos", shooter.leftPosition());
        telemetry.addData("Shooter R pos", shooter.rightPosition());

        telemetry.addData("Shooter Target RPM", "%.0f", target);
        telemetry.addData("Shooter L RPM (shooting+)", "%.0f", shooter.leftRpm());
        telemetry.addData("Shooter R RPM (shooting+)", "%.0f", shooter.rightRpm());

        telemetry.addData("At speed?", active != null && shooter.isAtSpeed(target, sequencer.getAtSpeedTolRpm()));
        telemetry.addData("State", sequencer.state());
        telemetry.addData("Tipper", tipper.isFiring() ? "FIRING" : "REST");

        telemetry.update();
    }
}
