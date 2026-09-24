package frc.robot.Auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * StaticPathfinder provides basic obstacle avoidance for the "Glide" feature.
 * Synchronized with Arena2026Rebuilt definitions.
 */
public class StaticPathfinder {

    public interface Obstacle {
        boolean isBlocking(Translation2d p1, Translation2d p2);

        Translation2d getCenter();

        double getSafeRadius();
    }

    public static class CircularObstacle implements Obstacle {
        public final Translation2d center;
        public final double radius;

        public CircularObstacle(Translation2d center, double radius) {
            this.center = center;
            this.radius = radius;
        }

        @Override
        public boolean isBlocking(Translation2d p1, Translation2d p2) {
            Translation2d d = p2.minus(p1);
            Translation2d f = p1.minus(center);
            double a = d.dot(d);
            double b = 2 * f.dot(d);
            double c = f.dot(f) - radius * radius;
            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0)
                return false;
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);
            return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1) || (t1 < 0 && t2 > 1);
        }

        @Override
        public Translation2d getCenter() {
            return center;
        }

        @Override
        public double getSafeRadius() {
            return radius + 0.6;
        }
    }

    public static class RectangularObstacle implements Obstacle {
        public final Translation2d center;
        public final double width;
        public final double height;
        public final Rotation2d rotation;

        public RectangularObstacle(Translation2d center, double width, double height, Rotation2d rotation) {
            this.center = center;
            this.width = width;
            this.height = height;
            this.rotation = rotation;
        }

        @Override
        public boolean isBlocking(Translation2d p1, Translation2d p2) {
            // Simplified: treat large rectangles as circles for detour calculation,
            // but use more precise intersection if needed.
            // For now, let's use a bounding circle check for simplicity in path avoiding.
            double rad = Math.sqrt(width * width + height * height) / 2.0;
            CircularObstacle proxy = new CircularObstacle(center, rad);
            return proxy.isBlocking(p1, p2);
        }

        @Override
        public Translation2d getCenter() {
            return center;
        }

        @Override
        public double getSafeRadius() {
            return Math.sqrt(width * width + height * height) / 2.0 + 0.6;
        }
    }

    private static final double IN_TO_M = 0.0254;

    private static final List<Obstacle> OBSTACLES = new ArrayList<>();
    static {
        // Trench Wall Constants
        double trenchWallDistX = (120.0 + 47.0 / 2.0) * IN_TO_M;
        double trenchWallDistY = (73.0 + 47.0 / 2.0 + 6.0) * IN_TO_M;
        double tw = 53.0 * IN_TO_M;
        double th = 12.0 * IN_TO_M;

        // Trench Walls (Symmetric assumptions)
        OBSTACLES.add(new RectangularObstacle(new Translation2d(8.27 - trenchWallDistX, 4.035 - trenchWallDistY), tw,
                th, new Rotation2d()));
        OBSTACLES.add(new RectangularObstacle(new Translation2d(8.27 + trenchWallDistX, 4.035 - trenchWallDistY), tw,
                th, new Rotation2d()));
        OBSTACLES.add(new RectangularObstacle(new Translation2d(8.27 - trenchWallDistX, 4.035 + trenchWallDistY), tw,
                th, new Rotation2d()));
        OBSTACLES.add(new RectangularObstacle(new Translation2d(8.27 + trenchWallDistX, 4.035 + trenchWallDistY), tw,
                th, new Rotation2d()));

        // Poles
        OBSTACLES.add(new RectangularObstacle(new Translation2d(42 * IN_TO_M, 159 * IN_TO_M), 2 * IN_TO_M, 47 * IN_TO_M,
                new Rotation2d()));
        OBSTACLES.add(new RectangularObstacle(new Translation2d((651 - 42) * IN_TO_M, 170 * IN_TO_M), 2 * IN_TO_M,
                47 * IN_TO_M, new Rotation2d()));

        // Hubs (central scoring structure: 47" x 47" ~ 1.19m x 1.19m)
        double hubSize = 47.0 * IN_TO_M;
        OBSTACLES.add(new RectangularObstacle(new Translation2d(4.60, 4.035), hubSize, hubSize, new Rotation2d()));
        OBSTACLES.add(new RectangularObstacle(new Translation2d(11.94, 4.035), hubSize, hubSize, new Rotation2d()));
    }

    private static Translation2d clampToField(Translation2d p) {
        double clampedX = Math.max(0.65, Math.min(15.89, p.getX()));
        double clampedY = Math.max(0.65, Math.min(7.56, p.getY()));
        return new Translation2d(clampedX, clampedY);
    }

    public static List<Pose2d> findPath(Pose2d start, Pose2d target) {
        List<Pose2d> path = new ArrayList<>();
        findPathRecursive(start.getTranslation(), target.getTranslation(), target.getRotation(), path, 0);
        path.add(target);
        return path;
    }

    private static void findPathRecursive(Translation2d p1, Translation2d p2, Rotation2d finalRot, List<Pose2d> out, int depth) {
        if (depth > 2) return;

        Obstacle blocker = null;
        for (Obstacle obs : OBSTACLES) {
            if (obs.isBlocking(p1, p2)) {
                blocker = obs;
                break;
            }
        }
        if (blocker == null) return;

        Translation2d line = p2.minus(p1);
        double lineLen = line.getNorm();
        if (lineLen < 1e-4) return;

        Translation2d normal = new Translation2d(-line.getY(), line.getX()).div(lineLen);
        double dist = Math.max(1.65, blocker.getSafeRadius());

        Translation2d d1 = clampToField(blocker.getCenter().plus(normal.times(dist)));
        Translation2d d2 = clampToField(blocker.getCenter().plus(normal.times(-dist)));

        double cost1 = p1.getDistance(d1) + d1.getDistance(p2);
        double cost2 = p1.getDistance(d2) + d2.getDistance(p2);
        Translation2d best = (cost1 <= cost2) ? d1 : d2;

        // Check if first segment needs detour
        findPathRecursive(p1, best, finalRot, out, depth + 1);
        out.add(new Pose2d(best, finalRot));
        // Check if second segment needs detour
        findPathRecursive(best, p2, finalRot, out, depth + 1);
    }
}
