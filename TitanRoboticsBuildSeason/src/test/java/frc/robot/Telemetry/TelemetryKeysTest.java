package frc.robot.Telemetry;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;
import java.nio.file.Files;
import java.util.HashSet;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.junit.jupiter.api.Test;

import frc.robot.Sim.SimDashboardKeys;

public class TelemetryKeysTest {

    @Test
    public void testAllKeysPopulatedAndWellFormed() {
        Set<String> keys = TelemetryKeys.allKeys();
        assertNotNull(keys);
        assertFalse(keys.isEmpty());

        for (String key : keys) {
            assertNotNull(key);
            assertFalse(key.trim().isEmpty(), "Key should not be empty");
            // Either starts with standard prefix or is an Auto key like "Auto Delay (seconds)"
            assertTrue(
                key.startsWith("Features/") ||
                key.startsWith("Driver/") ||
                key.startsWith("Operator/") ||
                key.startsWith("CoPilot/") ||
                key.startsWith("Build/") ||
                key.startsWith("Auto") ||
                key.startsWith("Current Action System") ||
                key.startsWith("Match/") ||
                key.startsWith("Coaching/") ||
                key.startsWith("JevAI/") ||
                key.startsWith("Strategy/") ||
                key.startsWith("AutoAim/") ||
                key.startsWith("Shooter/") ||
                key.startsWith("Power/") ||
                key.startsWith("PinWatchdog/") ||
                key.startsWith("SmartTunnel/") ||
                key.startsWith("Diagnostics/") ||
                key.startsWith("Scoreboard/"),
                "Unrecognized key namespace: " + key
            );
        }
    }

    @Test
    public void testElasticLayoutTopicsAreAccountedFor() {
        File layoutFile = new File("elastic-layout.json");
        if (!layoutFile.exists()) {
            layoutFile = new File("src/main/deploy/elastic-layout.json");
        }
        if (!layoutFile.exists()) {
            return; // In headless isolated test environment where layout is not in cwd
        }

        try {
            String content = Files.readString(layoutFile.toPath());
            Pattern pattern = Pattern.compile("\"topic\":\\s*\"/SmartDashboard/([^\"]+)\"");
            Matcher matcher = pattern.matcher(content);

            Set<String> layoutTopics = new HashSet<>();
            while (matcher.find()) {
                layoutTopics.add(matcher.group(1));
            }

            Set<String> registeredKeys = new HashSet<>(TelemetryKeys.allKeys());
            registeredKeys.addAll(SimDashboardKeys.allKeys());

            // Add standard engine-generated topics that are dynamically registered
            Set<String> dynamicPrefixes = Set.of(
                "Field", "swerve", "Alerts/", "Vision/", "Shooter/", "Intake/",
                "Simulation/", "Diagnostics/", "Scoreboard/", "Test/"
            );

            for (String topic : layoutTopics) {
                boolean isRegistered = registeredKeys.contains(topic);
                boolean isDynamic = dynamicPrefixes.stream().anyMatch(topic::startsWith);

                assertTrue(isRegistered || isDynamic,
                    "Topic from elastic-layout.json is not registered in TelemetryKeys or SimDashboardKeys: " + topic);
            }
        } catch (Exception e) {
            fail("Failed reading layout file: " + e.getMessage());
        }
    }
}
