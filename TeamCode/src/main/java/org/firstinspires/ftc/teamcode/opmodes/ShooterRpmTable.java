// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/ShooterRpmTable.java
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/*
 * The shooter's DISTANCE -> RPM table, and how distance to the AprilTag is measured.
 * This is not an OpMode. It is shared by two OpModes so they always agree:
 *
 *   - ShooterDistanceTuner fills in the table with test shots and saves it on the Robot Controller.
 *   - LimelightTagLockAutoRpm loads the saved table and sets the motor speed from it.
 *
 * DISTANCE = the distance along the floor from the spot directly under the center of the
 * AprilTag to the spot directly under the Limelight's lens, in inches. It comes from the
 * Limelight's 3D AprilTag data ("camera pose in target space"), so turning the camera servo
 * does not change it. The tag must be mounted upright (on a wall), like the field tags.
 *
 * The saved file is /sdcard/FIRST/settings/shooter_rpm_table.csv on the Robot Controller.
 * It is plain text, one point per line: distance in inches, then RPM (for example "48.0,3100").
 * Delete the file to go back to the built-in table in LimelightTagLockAutoRpm.
 */
public final class ShooterRpmTable {

    public static final String FILE_NAME = "shooter_rpm_table.csv";

    // Distances (inches) and the RPM for each, sorted from closest to farthest
    private final double[] distanceIn;
    private final double[] rpm;

    /** Makes a table from matching lists of distances and RPMs. They are sorted by distance. */
    public ShooterRpmTable(double[] distances, double[] rpms) {
        if (distances.length != rpms.length || distances.length == 0) {
            throw new IllegalArgumentException("Distance and RPM lists must be the same length and not empty");
        }
        Integer[] order = new Integer[distances.length];
        for (int i = 0; i < order.length; i++) order[i] = i;
        Arrays.sort(order, (a, b) -> Double.compare(distances[a], distances[b]));

        distanceIn = new double[distances.length];
        rpm = new double[rpms.length];
        for (int i = 0; i < order.length; i++) {
            distanceIn[i] = distances[order[i]];
            rpm[i] = rpms[order[i]];
        }
    }

    public int size() {
        return distanceIn.length;
    }

    public double distanceAt(int i) {
        return distanceIn[i];
    }

    public double rpmAt(int i) {
        return rpm[i];
    }

    /**
     * RPM for a distance, blending between the two nearest points. Closer than the first point
     * uses the first RPM; farther than the last point uses the last RPM.
     */
    public double rpmFor(double distance) {
        int last = distanceIn.length - 1;
        if (distance <= distanceIn[0]) return rpm[0];
        if (distance >= distanceIn[last]) return rpm[last];

        for (int i = 1; i <= last; i++) {
            if (distance <= distanceIn[i]) {
                double d0 = distanceIn[i - 1];
                double d1 = distanceIn[i];
                if (d1 <= d0) return rpm[i];   // two points at the same distance
                double fraction = (distance - d0) / (d1 - d0);
                return rpm[i - 1] + fraction * (rpm[i] - rpm[i - 1]);
            }
        }
        return rpm[last];
    }

    // =====================================================================================
    // Saving and loading
    // =====================================================================================

    /** The file on the Robot Controller: /sdcard/FIRST/settings/shooter_rpm_table.csv */
    public static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    /** Writes the table to the file, replacing what was there. */
    public void save() throws IOException {
        StringBuilder text = new StringBuilder();
        text.append("# Shooter DISTANCE -> RPM table, written by ShooterDistanceTuner\n");
        text.append("# distance_in,rpm\n");
        for (int i = 0; i < distanceIn.length; i++) {
            text.append(String.format(Locale.US, "%.1f,%.0f%n", distanceIn[i], rpm[i]));
        }
        ReadWriteFile.writeFileOrThrow(file(), text.toString());
    }

    /** The saved table, or null if there is no file or it has no usable points. */
    public static ShooterRpmTable loadOrNull() {
        File f = file();
        if (!f.exists()) return null;

        List<double[]> points = new ArrayList<>();
        for (String line : ReadWriteFile.readFile(f).split("\\r?\\n")) {
            line = line.trim();
            if (line.isEmpty() || line.startsWith("#")) continue;

            String[] parts = line.split(",");
            if (parts.length < 2) continue;
            try {
                double d = Double.parseDouble(parts[0].trim());
                double r = Double.parseDouble(parts[1].trim());
                if (d > 0 && r > 0) points.add(new double[] { d, r });
            } catch (NumberFormatException ignored) {
                // skip lines that are not two numbers
            }
        }
        if (points.isEmpty()) return null;

        double[] d = new double[points.size()];
        double[] r = new double[points.size()];
        for (int i = 0; i < points.size(); i++) {
            d[i] = points.get(i)[0];
            r[i] = points.get(i)[1];
        }
        return new ShooterRpmTable(d, r);
    }

    /** The table as Java code, in case you want to paste it into LimelightTagLockAutoRpm. */
    public String toJavaArrays() {
        StringBuilder d = new StringBuilder("TABLE_DISTANCE_IN = { ");
        StringBuilder r = new StringBuilder("TABLE_RPM         = { ");
        for (int i = 0; i < distanceIn.length; i++) {
            if (i > 0) { d.append(", "); r.append(", "); }
            d.append(String.format(Locale.US, "%.1f", distanceIn[i]));
            r.append(String.format(Locale.US, "%.0f", rpm[i]));
        }
        return d.append(" };\n").append(r).append(" };").toString();
    }

    // =====================================================================================
    // Distance to the tag
    // =====================================================================================

    /**
     * Floor distance from the spot under the tag's center to the spot under the Limelight lens,
     * in inches. Returns 0 if the Limelight sent no 3D data for this tag.
     *
     * In the tag's own coordinates (Limelight "target space"), X points sideways along the tag,
     * Y points down, and Z points straight out of the tag. For an upright tag, X and Z are both
     * level with the floor, so the floor distance is the length of (X, Z) and Y is the height.
     */
    public static double floorDistanceInches(LLResultTypes.FiducialResult tag) {
        Position p = cameraPositionInTagSpaceInches(tag);
        if (p == null) return 0.0;
        return Math.hypot(p.x, p.z);
    }

    /**
     * How far the Limelight lens is BELOW the tag's center, in inches (negative if above).
     * Shown on the Driver Station as a sanity check: it should match the real height difference.
     * If it looks like a distance instead, the 3D data is not coming through the way this code expects.
     */
    public static double cameraBelowTagInches(LLResultTypes.FiducialResult tag) {
        Position p = cameraPositionInTagSpaceInches(tag);
        return p == null ? 0.0 : p.y;
    }

    private static Position cameraPositionInTagSpaceInches(LLResultTypes.FiducialResult tag) {
        if (tag == null) return null;
        Pose3D pose = tag.getCameraPoseTargetSpace();
        if (pose == null || pose.getPosition() == null) return null;

        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        // All zeros means the Limelight did not send 3D data for this tag
        if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) return null;
        return p;
    }
}
