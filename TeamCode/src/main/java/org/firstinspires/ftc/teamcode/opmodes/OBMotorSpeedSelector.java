// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/OBMotorSpeedSelector.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Motor Speed Selector for a goBILDA 6000 RPM (5203 Yellow Jacket, 1:1) DC motor.
 *
 * Press a button once and the motor holds that speed until you press a different button.
 * Speed is closed-loop (setVelocity), so it holds RPM under load instead of just a % power.
 *
 * Controls (gamepad1):
 *  - A:         1500 RPM (25%)
 *  - B:         3000 RPM (50%)
 *  - X:         4500 RPM (75%)
 *  - Y:         6000 RPM (100% / full speed)
 *  - Dpad down: STOP (motor coasts down)
 *
 * Robot Config: name the motor "spinMotor" (or change MOTOR_NAME below).
 *
 * Note: 6000 RPM is the motor's no-load free speed at 12V. Under any load, or on a
 * low battery, it will top out a bit below that; the Y button then just runs it flat out.
 */
@TeleOp(name = "OB Motor Speed Selector", group = "Test")
public class OBMotorSpeedSelector extends OpMode {

    // =========================
    // Config (edit these as needed)
    // =========================
    private static final String MOTOR_NAME = "spinMotor";

    // goBILDA 6000 RPM Yellow Jacket: 28 encoder ticks per output shaft revolution (1:1 gearbox)
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;

    // Button speeds (RPM)
    private static final double RPM_A = 1500.0;
    private static final double RPM_B = 3000.0;
    private static final double RPM_X = 4500.0;
    private static final double RPM_Y = 6000.0;

    // Flip to REVERSE if the motor spins the wrong way
    private static final DcMotorSimple.Direction MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // Hardware
    private DcMotorEx motor;

    // State
    private double targetRpm = 0.0;
    private String activeButton = "none (stopped)";

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motor.setDirection(MOTOR_DIRECTION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // FLOAT so the motor coasts to a stop instead of slamming the brakes at high RPM
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0.0);

        telemetry.addLine("OB Motor Speed Selector ready");
        telemetry.addLine("A: 1500 | B: 3000 | X: 4500 | Y: 6000 RPM");
        telemetry.addLine("Dpad down: STOP");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Stop is checked first so it always wins if multiple buttons are held
        if (gamepad1.dpad_down) {
            targetRpm = 0.0;
            activeButton = "none (stopped)";
        } else if (gamepad1.a) {
            targetRpm = RPM_A;
            activeButton = "A";
        } else if (gamepad1.b) {
            targetRpm = RPM_B;
            activeButton = "B";
        } else if (gamepad1.x) {
            targetRpm = RPM_X;
            activeButton = "X";
        } else if (gamepad1.y) {
            targetRpm = RPM_Y;
            activeButton = "Y";
        }

        if (targetRpm <= 0.0) {
            // Cut power (coast) rather than commanding 0 velocity, which would actively brake
            motor.setPower(0.0);
        } else {
            motor.setVelocity(rpmToTicksPerSec(targetRpm));
        }

        double actualRpm = ticksPerSecToRpm(motor.getVelocity());

        telemetry.addData("Active button", activeButton);
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Actual RPM", "%.0f", actualRpm);
        telemetry.addData("% of max", "%.0f%%", (actualRpm / MAX_RPM) * 100.0);
        telemetry.addLine();
        telemetry.addLine("A: 1500 | B: 3000 | X: 4500 | Y: 6000 | Dpad down: STOP");
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
