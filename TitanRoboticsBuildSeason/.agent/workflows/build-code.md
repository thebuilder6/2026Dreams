---
description: How to build the robot code using the correct WPILib JDK
---

# Build Code Workflow

To build the robot code and avoid registry key/JDK errors, follow these steps:

// turbo
1. Set the `JAVA_HOME` environment variable and run the Gradle build:
```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'; .\gradlew build
```

2. If you need to run the code in simulation, use:
```powershell
$env:JAVA_HOME = 'C:\Users\Public\wpilib\2026\jdk'; .\gradlew simulateJava
```
