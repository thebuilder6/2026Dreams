package frc.robot.Data;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode,
 * returns default if not or if tuning mode is disabled.
 */
public class TunableNumber {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private double defaultValue;
  private DoubleSubscriber subscriber;
  private DoublePublisher publisher;
  private boolean hasDefault = false;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new TunableNumber
   * 
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.key = dashboardKey;
  }

  /**
   * Create a new TunableNumber with the default value
   * 
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   * 
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   * 
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (Constants.TUNING_MODE) {
      DoubleTopic topic = NetworkTableInstance.getDefault().getDoubleTopic(tableKey + "/" + key);
      
      // Close previous publisher if it exists to avoid leaks
      if (publisher != null) {
        publisher.close();
      }
      publisher = topic.publish();
      publisher.set(defaultValue);
      
      // We only subscribe if we are in tuning mode
      if (subscriber == null) {
        subscriber = topic.subscribe(defaultValue);
      }
    }
    hasDefault = true;
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   * 
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    }

    return Constants.TUNING_MODE ? subscriber.get() : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   * 
   * @param id Unique identifier for the caller to avoid multiple checks overlapping
   * @return True if the number has changed since the last time this method was called
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }
}
