package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Servo that tips a ball into the flywheels. */
public class Tipper {

    private static final double FIRE_POS = 0.25;
    private static final double REST_POS = 0.10;

    private final Servo servo;

    public Tipper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "tipperServo");
        rest();
    }

    public void fire() { servo.setPosition(FIRE_POS); }
    public void rest() { servo.setPosition(REST_POS); }

    public boolean isFiring() {
        return servo.getPosition() >= (FIRE_POS - 0.02);
    }
}
