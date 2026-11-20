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
 * Each press of A adds 100 RPM (one step per press; holding A does not keep adding).
 * Speed is closed-loop (setVelocity), so it holds RPM under load.
 *
 * Controls (gamepad1):
 *  - A: +100 RPM (caps at 6000 RPM)
 *
 * Press STOP on the Driver Station to stop the motor (it coasts down).
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
    private static final double MAX_RPM   = 6000.0;

    // Flip to REVERSE if the motor spins the wrong way
    private static final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // Hardware
    private DcMotorEx motor;

    // State
    private double targetRpm = START_RPM;
    private boolean prevA = false;

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
        telemetry.addLine("Starts at 2500 RPM on PLAY | A: +100 RPM");
        telemetry.update();
    }

    @Override
    public void start() {
        targetRpm = START_RPM;
        prevA = gamepad1.a; // don't count an A that was already held when PLAY was pressed
    }

    @Override
    public void loop() {
        // Edge detection: only add a step on the moment A goes from released -> pressed
        boolean a = gamepad1.a;
        if (a && !prevA) {
            targetRpm = Range.clip(targetRpm + STEP_RPM, 0.0, MAX_RPM);
        }
        prevA = a;

        motor.setVelocity(rpmToTicksPerSec(targetRpm));

        double actualRpm = ticksPerSecToRpm(motor.getVelocity());

        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Actual RPM", "%.0f", actualRpm);
        if (targetRpm >= MAX_RPM) telemetry.addLine("At max (6000 RPM)");
        telemetry.addLine();
        telemetry.addLine("A: +100 RPM");
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
