// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBMotorRpmStepUp.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Motor RPM Step Up for a goBILDA 6000 RPM (5203 Yellow Jacket, 1:1) DC motor.
 *
 * When you press PLAY the motor spins up to 2500 RPM.
 * Each press of A adds 100 RPM, each press of B takes away 100 RPM
 * (one step per press; holding a button does not keep stepping).
 * Speed is closed-loop (setVelocity), so it holds RPM under load.
 *
 * Controls (gamepad1):
 *  - A:         +100 RPM (caps at 6000 RPM)
 *  - B:         -100 RPM (bottoms out at 100 RPM; use Dpad down to fully stop)
 *  - Dpad down: STOP (motor coasts down)
 *  - A while stopped: restart at 2500 RPM
 *
 * Robot Config: name the motor "spinMotor" (or change MOTOR_NAME below).
 */
@TeleOp(name = "OB Motor RPM Step Up", group = "Test")
public class OBMotorRpmStepUp extends OpMode {

    // =========================
    // Config (edit these as needed)
    // =========================
    private static final String MOTOR_NAME = "spinMotor";

    // goBILDA 6000 RPM Yellow Jacket: 28 encoder ticks per output shaft revolution (1:1 gearbox)
    private static final double TICKS_PER_REV = 28.0;

    private static final double START_RPM = 2500.0;
    private static final double STEP_RPM  = 100.0;
    private static final double MIN_RPM   = 100.0;  // lowest B can step down to while running
    private static final double MAX_RPM   = 6000.0;

    // Flip to REVERSE if the motor spins the wrong way
    private static final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // Hardware
    private DcMotorEx motor;

    // State
    private double targetRpm = START_RPM;
    private boolean stopped = false;
    private boolean prevA = false;
    private boolean prevB = false;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motor.setDirection(MOTOR_DIRECTION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // FLOAT so the motor coasts to a stop instead of slamming the brakes at high RPM
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0.0);

        telemetry.addLine("OB Motor RPM Step Up ready");
        telemetry.addLine("Starts at 2500 RPM on PLAY");
        telemetry.addLine("A: +100 | B: -100 | Dpad down: STOP");
        telemetry.update();
    }

    @Override
    public void start() {
        targetRpm = START_RPM;
        stopped = false;
        // Don't count a button that was already held when PLAY was pressed
        prevA = gamepad1.a;
        prevB = gamepad1.b;
    }

    @Override
    public void loop() {
        // Edge detection: only step on the moment a button goes from released -> pressed
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean aPressed = a && !prevA;
        boolean bPressed = b && !prevB;
        prevA = a;
        prevB = b;

        if (gamepad1.dpad_down) {
            // Stop wins over everything else
            stopped = true;
        } else if (stopped) {
            // While stopped, A restarts at the starting speed; B does nothing
            if (aPressed) {
                targetRpm = START_RPM;
                stopped = false;
            }
        } else {
            if (aPressed) targetRpm += STEP_RPM;
            if (bPressed) targetRpm -= STEP_RPM;
            targetRpm = Range.clip(targetRpm, MIN_RPM, MAX_RPM);
        }

        if (stopped) {
            // Cut power (coast) rather than commanding 0 velocity, which would actively brake
            motor.setPower(0.0);
        } else {
            motor.setVelocity(rpmToTicksPerSec(targetRpm));
        }

        double actualRpm = ticksPerSecToRpm(motor.getVelocity());

        telemetry.addData("Status", stopped ? "STOPPED (press A to restart at 2500)" : "RUNNING");
        telemetry.addData("Target RPM", "%.0f", stopped ? 0.0 : targetRpm);
        telemetry.addData("Actual RPM", "%.0f", actualRpm);
        if (!stopped && targetRpm >= MAX_RPM) telemetry.addLine("At max (6000 RPM)");
        if (!stopped && targetRpm <= MIN_RPM) telemetry.addLine("At min (100 RPM) - Dpad down to stop");
        telemetry.addLine();
        telemetry.addLine("A: +100 | B: -100 | Dpad down: STOP");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (motor != null) motor.setPower(0.0);
    }

    // =========================
    // Helpers
    // =========================
    private static double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private static double ticksPerSecToRpm(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }
}
