# CompRobot2025 - FRC Team 4680

## Overview

This is the robot code for **FRC Team 4680's 2025 competition robot**. The robot is designed for the 2025 FIRST Robotics Competition game and features a sophisticated swerve drive system with multiple subsystems for game piece manipulation.

## Robot Capabilities

### Core Systems
- **Swerve Drive**: Advanced 4-wheel swerve drive with field-oriented control
- **Intake System**: Dual-roller intake with algae detection sensor
- **Elevator System**: Multi-position vertical elevator for game piece positioning
- **Wrist Mechanism**: Articulating wrist for precise game piece orientation
- **Launcher System**: High-speed launcher for scoring game pieces
- **Feeder System**: Controlled feeding mechanism for launcher
- **Vision System**: Integrated vision processing for autonomous navigation

### Game Strategy
The robot is designed to handle **algae game pieces** with autonomous routines for:
- **1, 2, and 3 Algae** autonomous sequences
- **Reef collection** and scoring
- **Lollipop intake** positioning
- **Processor station** interaction
- **Climbing capabilities**

## Technical Specifications

### Software Framework
- **Language**: Java 17
- **Framework**: WPILib 2025.3.1
- **Build System**: Gradle
- **Architecture**: Command-based robot programming

### Key Dependencies
- **YAGSL (Yet Another Generic Swerve Library)**: v2025.7.0
- **PathPlanner**: v2025.2.6 for autonomous path planning
- **REVLib**: REV Robotics motor controller integration
- **Phoenix 6**: CTRE motor controller support (v25.3.1)
- **PhotonVision**: Vision processing library
- **AdvantageKit**: Logging and replay framework

### Hardware Configuration
- **Team Number**: 4680
- **Drivetrain**: NEO motors with swerve modules
- **Controllers**: Dual Xbox controllers (Driver: Port 1, Operator: Port 0)
- **Sensors**: Algae detection via analog sensor

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java              # Main robot class
├── RobotContainer.java     # Command bindings and subsystem management
├── Main.java              # Entry point
├── Constants.java         # Robot constants and configuration
├── subsystems/            # Robot subsystems
│   ├── swervedrive/       # Swerve drive and vision
│   ├── IntakeSubsystem.java
│   ├── ElevatorSubsystem.java
│   ├── WristSubsystem.java
│   ├── LauncherSubsystem.java
│   └── FeederSubsystem.java
└── commands/              # Robot commands
    ├── swervedrive/       # Drive commands
    ├── IntakeCommand.java
    ├── ElevatorCommand.java
    ├── MoveWristCommand.java
    ├── LauncherCommand.java
    └── FeederCommand.java

src/main/deploy/
├── pathplanner/           # Autonomous paths and configurations
│   ├── autos/            # Autonomous routines (1Algae, 2Algae, 3Algae, etc.)
│   └── paths/            # Individual path segments
└── swerve/neo/           # Swerve drive configuration files
```

## Getting Started

### Prerequisites
- **WPILib 2025** installation
- **Java 17** JDK
- **Git** for version control
- **VS Code** with WPILib extension (recommended)

### Setup Instructions

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd CompRobot2025
   ```

2. **Open in VS Code**:
   ```bash
   code .
   ```

3. **Build the project**:
   ```bash
   ./gradlew build
   ```

4. **Deploy to robot**:
   ```bash
   ./gradlew deploy
   ```

### Development Workflow

#### Building and Testing
```bash
# Build the project
./gradlew build

# Run tests
./gradlew test

# Deploy to robot
./gradlew deploy

# Simulate robot code
./gradlew simulateJava
```

#### Key Gradle Tasks
- `build`: Compile and build the robot code
- `deploy`: Deploy code to the RoboRIO
- `simulateJava`: Run robot simulation
- `test`: Execute unit tests

## Robot Control

### Driver Controls (Xbox Controller - Port 1)
- **Left Stick**: Translation (X/Y movement)
- **Right Stick**: Rotation control
- **Left Bumper**: Reset gyro/field orientation
- **Right Trigger**: Elevator latch/climb sequence

### Operator Controls (Xbox Controller - Port 0)
- **Right Bumper**: Intake (ground/reef based on elevator position)
- **Left Trigger**: Lollipop intake
- **Right Trigger**: Launch sequence
- **Left Bumper**: Processor/eject mode
- **A Button**: Ground intake position
- **X Button**: L2 intake position  
- **Y Button**: L3 intake position
- **B Button**: Launch position

## Autonomous Modes

The robot includes several autonomous routines:

### Available Autonomous Sequences
- **1Algae**: Single algae collection and scoring
- **2Algae**: Two algae collection and scoring sequence
- **3Algae**: Three algae collection and scoring (primary competition auto)
- **SideAlgae**: Side-start algae collection routine
- **TestAuto**: Development and testing autonomous

### PathPlanner Integration
- Utilizes PathPlanner for smooth, constraint-based autonomous paths
- Named commands for coordinated subsystem actions
- Real-time path following with obstacle avoidance

## Robot Constants

Key robot parameters are defined in `Constants.java`:

### Drive Constants
- **Max Speed**: 12 ft/s (reduced from 16.6 ft/s for practice)
- **Robot Mass**: ~58 kg (128 lbs)
- **Wheel Lock Time**: 10 seconds

### Subsystem Positions
- **Wrist Positions**: Stow (0°), Ground Intake (32°), Launch (4.25°), etc.
- **Elevator Positions**: Ground (-0.5"), L2 (-24"), L3 (-54"), Launch (-44")
- **Timing**: Launch delays, intake timeouts, etc.

## Key Features

### Advanced Swerve Drive
- Field-oriented control with alliance-relative driving
- Heading snaps for precise robot orientation
- Configurable drive modes (angular velocity vs direct angle)

### Intelligent Game Piece Handling
- Sensor-based algae detection
- Automated intake sequences
- Coordinated multi-subsystem commands
- Position-based intake mode selection

### Robust Autonomous
- Multiple autonomous strategies
- Reliable path following
- Timeout-protected commands
- Modular command composition

## Competition History

This codebase represents the **"Finals winning code"** from the 2025 season, incorporating lessons learned and optimizations made throughout the competition season.

### Recent Updates (Auto-Rewrite-Attempt branch)
- Reduced max speed and ramp rates for practice safety
- Optimized elevator and wrist positions
- Improved autonomous timing and reliability
- Enhanced 3-algae autonomous routine
- Refined launch sequences and delays

## Troubleshooting

### Common Issues
1. **Deploy Failures**: Ensure robot is connected and powered on
2. **Simulation Issues**: Verify all vendor dependencies are installed
3. **Path Following**: Check PathPlanner settings and wheel circumference
4. **Control Issues**: Verify controller ports and bindings

### Debug Tools
- **SmartDashboard**: Real-time telemetry and debugging
- **AdvantageKit**: Detailed logging and replay
- **WPILib Simulation**: Desktop testing environment

## Contributing

### Code Style
- Follow WPILib Java conventions
- Use meaningful variable and method names
- Comment complex logic and magic numbers
- Update constants when tuning robot parameters

### Branch Strategy
- `main`: Stable competition code
- `FoggySpringWinter`: Current development branch
- Feature branches for major changes

## Team Information

- **Team Number**: 4680
- **Robot Name**: CompRobot2025
- **Competition Year**: 2025
- **Programming Language**: Java

## License

This project is licensed under the WPILib BSD License. See `WPILib-License.md` for details.

---

*Built with ❤️ by FRC Team 4680*
