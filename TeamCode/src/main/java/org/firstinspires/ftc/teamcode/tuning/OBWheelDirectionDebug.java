// TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/OBWheelDirectionDebug.java
package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.oysterbay.subsystems.MecanumDrivetrain;

/**
 * Debug opmode: run each wheel forward independently using A/B/X/Y,
 * and run the same wheel backward using the D-pad.
 *
 * Mapping (Gamepad1):
 *  Forward:
 *   - A -> Front Left
 *   - B -> Front Right
 *   - X -> Back Left
 *   - Y -> Back Right
 *
 *  Reverse:
 *   - Dpad Left  -> Front Left
 *   - Dpad Right -> Front Right
 *   - Dpad Down  -> Back Left
 *   - Dpad Up    -> Back Right
 *
 * Uses the same motor directions as RobotStructure (TeleOp).
 */
@TeleOp(name = "OB Debug: Wheel Dir (Direct Map)", group = "OB")
public class OBWheelDirectionDebug extends OpMode {

    private static final double TEST_POWER = 0.35;

    private MecanumDrivetrain drive;

    @Override
    public void init() {
        // Match RobotStructure: +power = forward for all wheels (right side reversed)
        drive = new MecanumDrivetrain(hardwareMap, MecanumDrivetrain.ReversedSide.RIGHT);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Wheel Direction Debug Ready (Direct Map)");
        telemetry.addLine("Hold A/B/X/Y to run one wheel forward.");
        telemetry.addLine("Hold Dpad to run the same wheel backward.");
        telemetry.addLine("A=FL, B=FR, X=BL, Y=BR");
        telemetry.addLine("DpadLeft=FL-, DpadRight=FR-, DpadDown=BL-, DpadUp=BR-");
        telemetry.update();
    }

    @Override
    public void loop() {
        double fl = wheelPower(gamepad1.a, gamepad1.dpad_left);
        double fr = wheelPower(gamepad1.b, gamepad1.dpad_right);
        double bl = wheelPower(gamepad1.x, gamepad1.dpad_down);
        double br = wheelPower(gamepad1.y, gamepad1.dpad_up);

        drive.setWheelPowers(fl, fr, bl, br);

        telemetry.addLine("Hold one button at a time for clean results.");
        telemetry.addData("FL (A / DpadLeft)",  fl);
        telemetry.addData("FR (B / DpadRight)", fr);
        telemetry.addData("BL (X / DpadDown)",  bl);
        telemetry.addData("BR (Y / DpadUp)",    br);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }

    /** Reverse overrides forward if both are held. */
    private static double wheelPower(boolean forward, boolean reverse) {
        if (reverse) return -TEST_POWER;
        if (forward) return TEST_POWER;
        return 0.0;
    }
}
