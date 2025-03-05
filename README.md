# 🔥🌊 TEAM 7221 REEFSCAPE 2025 🌊🔥

```
           _____  __  __   ___   __   _____  _____  _____  _   _ 
          |_   _||  \/  | / _ \  | | |  _  || ____|/ ___|| | | |
            | |  | |\/| || | | | | | | |_| || |__  \___ \| | | |
            | |  | |  | || |_| | | | |  _  ||  __|  ___) | |_| |
            |_|  |_|  |_| \___/  |_| |_| |_||_|    |____/ \___/ 
 _____  _____  _____  _____  _____ _____  _____  ____  _____  _____      
|  __ \|  ___|  ___|/ ____|/ ____|  _  ||  _  |/ __ \|  ___|  ___|     
| |__) | |__  | |__ | |    | (___ | |_| || |_| | |  | | |__  | |__     
|  _  /|  __| |  __|| |     \___ \|  _  ||  _  | |  | |  __| |  __|    
| | \ \| |___ | |___| |____ ____) | | | || | | | |__| | |___ | |___    
|_|  \_\_____|\_____/\_____/|_____/|_| |_||_| |_|\____/|_____|_____|   
```

![Team7221](https://img.shields.io/badge/Team-7221-blue)
![Season](https://img.shields.io/badge/Season-2025-orange)
![Game](https://img.shields.io/badge/Game-Reefscape-green)
![Build](https://img.shields.io/badge/Build-Passing-brightgreen)
![Java](https://img.shields.io/badge/Java-17-red)
![WPILib](https://img.shields.io/badge/WPILib-2025.3.1-purple)

> "Dominate the Reef. Control the Game. CRUSH THE COMPETITION!"

---

## 📋 Table of Contents

- [🤖 Robot Overview](#-robot-overview)
- [⚙️ Technical Specifications](#️-technical-specifications)
- [🏗️ System Architecture](#️-system-architecture)
- [🚀 Getting Started](#-getting-started)
- [🎮 Operating the Robot](#-operating-the-robot)
- [🛠️ Development Guide](#️-development-guide)
- [🏆 Competition Strategy](#-competition-strategy)
- [❓ Troubleshooting](#-troubleshooting)
- [👥 Team Information](#-team-information)
- [📸 Robot in Action](#-robot-in-action)

---

## 🤖 Robot Overview

Our 2025 competition robot, **REEF DOMINATOR**, is a precision-engineered machine built to excel in FRC Reefscape. Combining defensive capability with offensive prowess, our bot features enhanced maneuverability, intelligent ball control, and a hook mechanism for endgame strategy.

### ✨ Key Features

#### 🔄 DRIVETRAIN SYSTEM
- **Base Chassis**: Modified AM14U5 with 6-wheel tank drive
- **Center Drop**: Improved traction and zero-turn capability
- **Motor Setup**: 4x NEO brushless motors with 16:1 gear ratio (UPGRADED from 8.45:1)
- **Control Style**: Tank hardware with arcade-style controls for intuitive handling
- **Speed Modes**:
  - **TURBO**: 100% power for maximum speed (⚡ 12.7 ft/sec)
  - **NORMAL**: 85% power for balanced performance
  - **PRECISION**: 35% power for fine control

#### 🦾 BALL CONTROL ARM
- **Front-mounted**: Positioned for optimal ball acquisition
- **Rotation**: 180° powered by NEO brushless motor
- **Gripper**: Dual-wheel intake with variable speed control
- **Sensors**: Ultrasonic detection + limit switches
- **Positions**:
  - **HOME**: Tucked position for defense/driving
  - **PICKUP**: Floor-level ball acquisition
  - **SCORE**: Elevated position for ball release

#### 🪝 HOOK SYSTEM
- **Mechanism**: Linear actuator with J-hook end effector
- **Extension**: 6" stroke for optimal barge engagement
- **Control**: Precision deployment with limit switch detection
- **Safety**: Current-limiting and emergency stop capabilities

#### 👁️ VISION SYSTEM
- **Perception**: PhotonVision with ball tracking and AprilTag support
- **Camera**: High-FPS USB camera with wide field of view
- **Processing**: Real-time target acquisition and distance calculation
- **Modes**:
  - **BALL TRACKING**: Color/shape-based ball detection
  - **APRILTAG**: Field positioning using AprilTags
  - **DRIVER MODE**: Direct camera feed for human control

---

## ⚙️ Technical Specifications

### 📏 Physical Dimensions
- **Width**: 27 inches (bumpers included)
- **Length**: 32 inches (bumpers included)
- **Height**: 24 inches (arm stowed)
- **Weight**: 119 lbs (with battery)

### ⚡ Electrical System
- **Control System**: roboRIO 2.0
- **Motor Controllers**: 
  - 4x REV Spark MAX (drivetrain)
  - 1x REV Spark MAX (arm)
  - 1x REV Spark MAX (gripper)
  - 1x REV Spark MAX (hook)
- **Power Distribution**: REV PDP with digital fusing
- **Battery**: 12V 18Ah lead-acid
- **Sensors**:
  - NEO integrated encoders (all drive motors)
  - 2x Limit switches (arm)
  - 2x Limit switches (hook)
  - 1x Ultrasonic sensor (ball detection)
  - 1x USB Camera (vision system)

### 💾 Software Stack
- **Language**: Java
- **Framework**: WPILib 2025.3.1
- **Architecture**: Command-based programming
- **Control Theory**:
  - PID control for arm positioning
  - Slew rate limiters for smooth acceleration
  - Feedforward control for drivetrain
  - Encoder-based odometry for position tracking
- **Vision**: PhotonVision with multi-pipeline support
- **Autonomous**: Path following with encoder feedback

---

## 🏗️ System Architecture

Our robot code is structured using the command-based programming paradigm, separating robot functionality into Subsystems and Commands.

```
📁 src/main/java/frc/robot/
 ├── 📁 commands/         # Commands organized by subsystem
 │    ├── 📁 autonomous/  # Autonomous routines
 │    ├── 📁 hook/        # Hook system commands
 │    ├── 📁 vision/      # Vision processing commands
 │    └── 📄 BallControlCommands.java  # Ball arm control
 │    
 ├── 📁 subsystems/       # Core robot subsystems
 │    ├── 📄 DriveSubsystem.java       # Tank drive with arcade control
 │    ├── 📄 BallArmSubsystem.java     # Ball control arm
 │    ├── 📄 HookSubsystem.java        # Barge hook system
 │    └── 📄 VisionSubsystem.java      # Camera and targeting
 │
 ├── 📄 Constants.java    # Centralized robot constants
 ├── 📄 Main.java         # Program entry point
 ├── 📄 Robot.java        # Main robot class
 └── 📄 RobotContainer.java  # Command binding and initialization
```

### ⚡ Subsystems Overview

#### 🚗 DriveSubsystem
- Handles tank drive mechanics with arcade-style control
- Implements encoder-based position tracking
- Provides multiple drive modes (Turbo, Normal, Precision)
- Uses slew rate limiting for smooth acceleration

#### 🦾 BallArmSubsystem
- Controls arm position with PID feedback
- Manages gripper for ball collection and release
- Implements gravity compensation for smooth movement
- Uses ultrasonic sensing for ball detection

#### 🪝 HookSubsystem
- Controls linear actuator for hook extension/retraction
- Implements limit switch detection for end positions
- Provides current monitoring for motor protection
- Offers emergency stop functionality

#### 👁️ VisionSubsystem
- Integrates with PhotonVision for target processing
- Manages multiple vision pipelines
- Calculates target distance and position
- Supports driver camera mode

---

## 🚀 Getting Started

### Prerequisites
- **Java Development Kit** (JDK) 17+
- **WPILib 2025 Installation** (complete with VSCode)
- **REV Hardware Client** (for Spark MAX configuration)
- **Git** (for version control)
- **PhotonVision** (for vision processing)

### Initial Setup

1. **Clone the repository**
```bash
# Clone with SSH
git clone git@github.com:Team7221/Team7221-Reefscape-2025.git

# OR clone with HTTPS
git clone https://github.com/Team7221/Team7221-Reefscape-2025.git

# Navigate to the project directory
cd Team7221-Reefscape-2025
```

2. **Open the project in VSCode**
```bash
code .
```

3. **Install vendor dependencies**
   - Open the WPILib Command Palette (Shift+Ctrl+P)
   - Select "Manage Vendor Libraries"
   - Install the following dependencies:
     - REV Robotics (for Spark MAX)
     - PhotonVision (for vision processing)
     - PathPlanner (for autonomous routines)

4. **Configure CAN IDs**
   - Using REV Hardware Client, ensure all motor controllers have unique CAN IDs:
     - Left Front Drive: ID 1
     - Right Front Drive: ID 2
     - Left Back Drive: ID 3
     - Right Back Drive: ID 4
     - Ball Arm Motor: ID 5
     - Gripper Motor: ID 6
     - Hook Motor: ID 7

5. **Build the project**
```bash
# Using Gradle directly
./gradlew build

# Or use WPILib VSCode extension
# (Ctrl+Shift+P -> WPILib: Build Robot Code)
```

### Deploying to the Robot

1. **Connect to the robot**
   - Ensure you're connected to the robot's network
   - Verify roboRIO is powered on and accessible

2. **Deploy the code**
```bash
# Using Gradle
./gradlew deploy

# Or use WPILib VSCode shortcut
# (Ctrl+Shift+P -> WPILib: Deploy Robot Code)
```

3. **Verify deployment**
   - Check Driver Station for successful connection
   - Monitor console output for initialization messages
   - Verify all subsystems report "READY" status

---

## 🎮 Operating the Robot

### Driver Controls (Main Controller)
- **Left Joystick Y-Axis**: Forward/Backward Movement
- **Right Joystick X-Axis**: Turning
- **Right Bumper**: TURBO MODE (100% power)
- **Left Bumper**: PRECISION MODE (35% power)
- **Start Button**: Emergency Reset

### Operator Controls (Secondary Controller)
- **A Button**: Ball Pickup Sequence
- **Y Button**: Ball Scoring Sequence
- **B Button**: Arm Home Position
- **X Button**: Auto Ball Tracking
- **Left Trigger**: Release Ball (variable speed)
- **Right Trigger**: Intake Ball (variable speed)
- **Left Joystick Y-Axis**: Manual Arm Control
- **Back Button**: Extend Hook
- **Start Button**: Retract Hook
- **Right Stick Button**: Full Hook Cycle
- **Left Bumper + Right Bumper**: Emergency Stop

### 🧠 Autonomous Modes
1. **Ball Collection Auto**: Navigates field to collect balls
2. **Ball Hoarding Auto**: Collects multiple balls from field
3. **Reefscape Auto**: Optimized routine for Reefscape game
4. **Drive 1 Meter**: Simple forward drive
5. **Square Autonomous**: Drives in a square pattern (testing)
6. **Do Nothing**: Stays still (default)

Select the autonomous mode via the SmartDashboard before the match starts.

---

## 🛠️ Development Guide

### Coding Standards
- **Package Structure**: Follow the existing package structure
- **Naming Conventions**: 
  - Classes: PascalCase
  - Methods/Variables: camelCase
  - Constants: UPPER_SNAKE_CASE
- **Comments**: Document all methods and complex logic
- **Error Handling**: Use try-catch for potential failures

### Adding New Features

#### 1. Create a new Subsystem (if needed)
```java
public class NewSubsystem extends SubsystemBase {
    // Define hardware
    private final SparkMax m_motor;
    
    // Constructor
    public NewSubsystem() {
        m_motor = new SparkMax(Constants.NEW_MOTOR_ID, MotorType.kBrushless);
        // Configure hardware
    }
    
    // Methods
    public void doSomething() {
        // Implementation
    }
    
    @Override
    public void periodic() {
        // Periodic updates (runs every 20ms)
    }
}
```

#### 2. Create Commands for the subsystem
```java
public class NewCommand extends Command {
    private final NewSubsystem m_subsystem;
    
    public NewCommand(NewSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        // Called once when command starts
    }
    
    @Override
    public void execute() {
        // Called repeatedly during command execution
    }
    
    @Override
    public boolean isFinished() {
        // Return true when command should end
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Called once when command ends
    }
}
```

#### 3. Bind the command to controls in Robot.java
```java
// Add to configureButtonBindings() method
new Trigger(() -> operatorController.getAButton())
    .onTrue(new NewCommand(m_newSubsystem));
```

### Testing Your Code

1. **Simulation**:
   - Use WPILib Simulation feature to test code without hardware
   - Test with "WPILib: Start Simulation" command in VSCode

2. **Unit Testing**:
   - Write JUnit tests for critical components
   - Run with `./gradlew test`

3. **Hardware Testing**:
   - Always test in a safe, controlled environment
   - Start with reduced power/speed settings
   - Test one subsystem at a time before integration

---

## 🏆 Competition Strategy

### Offensive Play
1. **Ball Control**:
   - Use Auto Ball Tracking to quickly acquire game pieces
   - Store balls securely and transport to scoring zones
   - Time ball releases for maximum point impact

2. **Barge Manipulation**:
   - Use hook system to manipulate the barge for team advantage
   - Secure hook attachment with proper alignment
   - Coordinate with alliance partners for synchronized barge pulls

### Defensive Play
1. **Area Denial**:
   - Position bot to block opponent scoring paths
   - Use precision mode for accurate blocking positions
   - Maintain awareness of opponent positions

2. **Ball Hoarding**:
   - Collect balls to deny opponents scoring opportunities
   - Use arm system to quickly intake loose balls
   - Protect collected game pieces with defensive positioning

### Endgame Strategy
1. **Barge Connection**:
   - Position near a barge with ~20 seconds remaining
   - Use hook deployment sequence for secure attachment
   - Coordinate with alliance to maximize barge positioning

2. **Final Scoring**:
   - Release remaining balls for last-second points
   - Position for defensive blocking if no balls remain
   - Use turbo mode to reach strategic positions quickly

---

## ❓ Troubleshooting

### Common Issues

#### 1. Robot doesn't move
- **Check**: Battery voltage (should be >12V)
- **Check**: Breakers and PDP status
- **Check**: CAN connections and motor controller LEDs
- **Check**: Driver Station for errors
- **Solution**: Reset roboRIO if all hardware checks pass

#### 2. Arm movement issues
- **Check**: Limit switch functionality
- **Check**: Encoder readings (SmartDashboard)
- **Check**: Arm motor current draw (should be <30A)
- **Solution**: Re-zero arm encoder with `resetArmEncoder()` command

#### 3. Ball detection problems
- **Check**: Ultrasonic sensor connections
- **Check**: Distance readings in SmartDashboard
- **Check**: Sensor mounting position
- **Solution**: Adjust `BALL_DETECTION_THRESHOLD_INCHES` in Constants.java

#### 4. Motor controller errors
- **Check**: CAN termination resistors
- **Check**: Power connections
- **Check**: Update firmware in REV Hardware Client
- **Solution**: Reset all motor controllers and reboot roboRIO

#### 5. Code deployment failures
- **Check**: Network connection to roboRIO
- **Check**: Build errors in VSCode
- **Check**: Team number in `build.gradle`
- **Solution**: Generate fresh project and migrate code if needed

---

## 👥 Team Information

### Team 7221 - The Vikings Robotics
- **High School**: North High School
- **Location**: Minneapolis, MN
- **Founded**: 2018
- **Team Motto**: "Innovate. Build. Dominate."

### Contact Information
- **Email**: team7221@schooldistrict.edu
- **Website**: https://team7221.org
- **GitHub**: https://github.com/Team7221

### Our Values
- **Innovation**: We embrace creative solutions and cutting-edge technology
- **Teamwork**: We succeed through collaboration and communication
- **Excellence**: We strive for perfection in design and implementation
- **Sportsmanship**: We compete with respect and integrity
- **Passion**: We bring enthusiasm to everything we do

---

## 📸 Robot in Action

```
   /\         _____        /\       
  /  \       |  ___|      /  \      
 /    \      | |___      /    \     
/      \     |  ___|    /      \    
\      /     | |___     \      /    
 \    /      |_____|     \    /     
  \  /                    \  /      
   \/                      \/       
⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜
⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜
⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜⬜
      _____                         
     |_____|                        
   ____|_|____                      
  |    ___    |                     
  |   |___|   |                     
  |___________|                     
     |     |                        
    /|     |\                       
   / |_____| \                      
  /___________\                     
```

Watch our robot in action at the regional competitions:
- 🎥 [Team 7221 - Reefscape Robot Reveal](https://youtube.com/teamvikings)
- 🎥 [Minneapolis Regional Highlights](https://youtube.com/teamvikings)
- 🎥 [Robot Testing & Development Process](https://youtube.com/teamvikings)

---

> "This README represents our team's ongoing development. Code is continuously improved, and our robot evolves with each iteration. Join us on this journey as we push the boundaries of what's possible in FRC Reefscape!"

📅 Last Updated: March 2025  
🤖 Team 7221 - Vikings Robotics
