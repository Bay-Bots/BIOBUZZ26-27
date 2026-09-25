package org.firstinspires.ftc.teamcode.oysterbay.subsystems;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Accumulates sightings of two candidate AprilTags over several frames and decides between them.
 *
 * Decision rule:
 *  1) a tag must be seen >= minSeenFrames to count
 *  2) if both count: pick the higher "seen" count; tie-break by closer Z if both have a pose
 *  3) if still tied: choose tag 2 (so it doesn't "fall into default")
 *  4) if neither counts: null (caller runs its default plan)
 */
public class TagVote {

    private final int tag1Id;
    private final int tag2Id;
    private final int minSeenFrames;

    private int seen1 = 0;
    private int seen2 = 0;
    private double bestZ1 = Double.POSITIVE_INFINITY;
    private double bestZ2 = Double.POSITIVE_INFINITY;

    public TagVote(int tag1Id, int tag2Id, int minSeenFrames) {
        this.tag1Id = tag1Id;
        this.tag2Id = tag2Id;
        this.minSeenFrames = minSeenFrames;
    }

    /** Count one frame of detections. Detections are counted by ID even when ftcPose is null. */
    public void record(List<AprilTagDetection> dets) {
        for (AprilTagDetection d : dets) {
            if (d == null) continue;
            if (d.id == tag1Id) {
                seen1++;
                if (d.ftcPose != null) bestZ1 = Math.min(bestZ1, d.ftcPose.z);
            } else if (d.id == tag2Id) {
                seen2++;
                if (d.ftcPose != null) bestZ2 = Math.min(bestZ2, d.ftcPose.z);
            }
        }
    }

    public int seen1() { return seen1; }
    public int seen2() { return seen2; }
    public double bestZ1() { return bestZ1; }
    public double bestZ2() { return bestZ2; }

    /** @return tag1Id, tag2Id, or null if neither was seen confidently */
    public Integer decide() {
        boolean ok1 = seen1 >= minSeenFrames;
        boolean ok2 = seen2 >= minSeenFrames;

        if (!ok1 && !ok2) return null;
        if (ok1 != ok2) return ok1 ? tag1Id : tag2Id;

        if (seen1 != seen2) return seen1 > seen2 ? tag1Id : tag2Id;

        if (bestZ1 < Double.POSITIVE_INFINITY && bestZ2 < Double.POSITIVE_INFINITY) {
            return (bestZ1 <= bestZ2) ? tag1Id : tag2Id;
        }
        return tag2Id;
    }

    /** Human-readable plan label for a decision. */
    public String label(Integer decision) {
        if (decision == null) return "TAG3 (default)";
        return decision == tag1Id ? "TAG1" : "TAG2";
    }
}
