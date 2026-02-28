package org.team9140.frc2026;

import java.io.IOException;
import java.nio.file.Path;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;

public class FieldConstants {

    public static final FieldType fieldType = FieldType.ANDYMARK;
    public static final double FIELD_LENGTH = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
    public static final double FIELD_WIDTH = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

    public static class Hub {

        public static final double width = 1.1938;
        public static final double height = 1.8288;

        public static final Pose2d CENTER_POINT = new Pose2d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
                FIELD_WIDTH / 2.0, new Rotation2d(0));
        public static final Pose2d RED_CENTER_POINT = new Pose2d(
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
                FIELD_WIDTH / 2.0, new Rotation2d(0));
    }

    public static class Tower {

        public static final double innerOpeningWidth = 0.81915;
        public static final double frontFaceX = 1.105154;

        public static final Pose2d CENTER_POINT = new Pose2d(
                frontFaceX, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY(), new Rotation2d(0));

        public static final Pose2d LEFT_UPRIGHT = new Pose2d(
                frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                        + innerOpeningWidth / 2
                        + 0.01905,
                new Rotation2d(0));

        public static final Pose2d RIGHT_UPRIGHT = new Pose2d(
                frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                        - innerOpeningWidth / 2
                        - 0.01905,
                new Rotation2d(0));

        public static final Pose2d RED_CENTER_POINT = new Pose2d(
                FIELD_LENGTH - frontFaceX,
                AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY(),
                new Rotation2d(0));

        public static final Pose2d RED_LEFT_UPRIGHT = new Pose2d(
                FIELD_LENGTH - frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                        + innerOpeningWidth / 2
                        + 0.01905,
                new Rotation2d(0));
        public static final Pose2d RED_RIGHT_UPRIGHT = new Pose2d(
                FIELD_LENGTH - frontFaceX,
                (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                        - innerOpeningWidth / 2
                        - 0.01905,
                new Rotation2d(0));
    }

    public static class Lines {
        public static final double BLUE_ALLIANCE_ZONE = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get()
                .getX();
        public static final double RED_ALLIANCE_ZONE = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get()
                .getX();
    }

    public static class FeedingPositions {
        public static final Pose2d FEEDING_POS_LOWER = new Pose2d(2, 2, new Rotation2d());
        public static final Pose2d FEEDING_POS_UPPER = new Pose2d(2, 6, new Rotation2d());
        public static final Pose2d FEEDING_POS_LOWER_RED = new Pose2d(14.540988, 2, new Rotation2d());
        public static final Pose2d FEEDING_POS_UPPER_RED = new Pose2d(14.540988, 6, new Rotation2d());
    }

    public enum FieldType {
        ANDYMARK("andymark"),
        WELDED("welded");

        private final String jsonFolder;

        FieldType(String jsonFolder) {
            this.jsonFolder = jsonFolder;
        }

        public String getJsonFolder() {
            return jsonFolder;
        }
    }

    public enum AprilTagLayoutType {
        OFFICIAL("2026-official"),
        NONE("2026-none"),
        HUB("2026-hub"),
        OUTPOST("2026-outpost"),
        TOWER("2026-tower");

        private final String name;
        private volatile AprilTagFieldLayout layout;
        private volatile String layoutString;

        AprilTagLayoutType(String name) {
            this.name = name;
        }

        public AprilTagFieldLayout getLayout() {
            if (layout == null) {
                synchronized (this) {
                    if (layout == null) {
                        try {
                            Path p = Path.of(
                                    Filesystem.getDeployDirectory().getPath(),
                                    "apriltags",
                                    fieldType.getJsonFolder(),
                                    name + ".json");
                            layout = new AprilTagFieldLayout(p);
                            layoutString = new ObjectMapper().writeValueAsString(layout);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    }
                }
            }
            return layout;
        }

        public String getLayoutString() {
            if (layoutString == null) {
                getLayout();
            }
            return layoutString;
        }
    }
}
