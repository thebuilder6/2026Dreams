package frc.robot.Auto;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Annotation to mark a class as an autonomous mission that should be
 * automatically added to the mission chooser.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface AutoMission {
    /**
     * The display name for the mission in the chooser.
     */
    String name();
}
