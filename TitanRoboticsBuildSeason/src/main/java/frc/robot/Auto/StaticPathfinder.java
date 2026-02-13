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

        // Hubs (Assume ramps = true for max safety)
        double hw = 47 * IN_TO_M;
        double hh = 217 * IN_TO_M;
        OBSTACLES.add(new RectangularObstacle(new Translation2d(4.60, 4.035), hw, hh, new Rotation2d()));
        OBSTACLES.add(new RectangularObstacle(new Translation2d(11.94, 4.035), hw, hh, new Rotation2d()));
    }

    public static List<Pose2d> findPath(Pose2d start, Pose2d target) {
        List<Pose2d> path = new ArrayList<>();
        Translation2d p1 = start.getTranslation();
        Translation2d p2 = target.getTranslation();

        Obstacle blocker = null;
        for (Obstacle obs : OBSTACLES) {
            if (obs.isBlocking(p1, p2)) {
                blocker = obs;
                break;
            }
        }

        if (blocker == null) {
            path.add(target);
            return path;
        }

        // Generate detour
        Translation2d line = p2.minus(p1);
        Translation2d normal = new Translation2d(-line.getY(), line.getX()).div(line.getNorm());
        double dist = blocker.getSafeRadius();

        Translation2d d1 = blocker.getCenter().plus(normal.times(dist));
        Translation2d d2 = blocker.getCenter().plus(normal.times(-dist));

        Translation2d mid = p1.plus(p2).div(2.0);
        Translation2d best = (d1.getDistance(mid) < d2.getDistance(mid)) ? d1 : d2;

        path.add(new Pose2d(best, target.getRotation()));
        path.add(target);
        return path;
    }
}
