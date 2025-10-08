# 🌐 FGCVEN25 — FIRST Global Challenge 2025

Welcome to **FGCVEN25**, the Team Venezuela's repository for the **2025 FIRST Global Challenge**!  
Here you’ll find all of our robot code and documentation developed for the competition.
[![Ask Deepwiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/aschvley/ATTENDIX)

> 🤖 *This project represents teamwork, innovation, and STEM applied to robotics at an international level.*

---

## 🎯 Project Purpose
The **FIRST Global Challenge (FGC)** is an annual robotics competition where high school teams from around the world collaborate and compete in solving themed engineering challenges.

The purpose of this repository is to:
- 🛠️ Host our **robot control software**.
- 📄 Centralize **documentation, notes, and strategies** for the challenge.
- 📂 Organize everything from SDK dependencies to team-written code.
- 🌍 Provide a **transparent knowledge base** that future teams can build upon.

---

## 📂 Repository Structure
```
FGCVEN25/
├── 📂 FtcRobotController/	 # Base SDK & FTC-provided libraries (foundation project)
│   ├── 📂 src/main		     # SDK logic + mirrored team code for in-app testing
│	└── ⚙️ build.gradle		 # App-level Gradle config for the Robot Controller module
│
├── 📂 TeamCode/      		 # Main folder for team-written robot code (teleop, controllers, utils)
│   ├── 📂 lib		 		 # External libraries / JARs
│   │	└── 📦 opmodeannotation.jar
│   ├── 📂 src/main		 	 # Custom team code (OpModes, subsystems, utilities)
│   │   └── 📂 java/org/firstinspires/ftc/teamcode  
│   │       └── 📂 TeleOp  	 # TeleOp OpModes for driver-controlled robot operation
│   │           ├── 📂 controllers 		 # Controller mapping classes for drivers
│   │           │   ├── 📄 AcceleratorController.java
│   │           │   ├── 📄 DistanceSensorController.java
│   │           │   ├── 📄 ExtendController.java
│   │           │   ├── 📄 FunnelController.java
│   │           │   ├── 📄 HangingController.java
│   │           │   ├── 📄 HugController.java
│   │           │   ├── 📄 IntakeController.java
│   │           │   └── 📄 RampController.java
│   │           ├── 📄 RobotMap.java 	 # Hardware mapping & constants
│   │           └── 📄 TeleOpMain.java 	 # Main teleop class executed on robot
│   └── ⚙️ build.gradle		 # Gradle config for compiling team code and linking with the controller app
│
├── 📂 gradle/            	 	  	 # Gradle wrapper folder (build system helper)
│   ├── 📦 gradle-wrapper.jar  		 # JAR used to bootstrap Gradle
│   └── 📝 gradle-wrapper.properties 	 # Configuration for Gradle wrapper
│
├── 📂 libs/           		 # Placeholder for external libraries (currently empty)
│   └── 📝 README.md     	 # Instructions for adding third-party libraries
│
├── .gitignore        	     # Files ignored by git
├── 📜 LICENSE             	 # Open source license (BSD-3-Clause-Clear)
├── 📝 README.md 			 # This file!
├── ⚙️ build.common.gradle 	   # Shared Gradle configuration
├── ⚙️ build.dependencies.gradle	# Dependency management
├── ⚙️ build.gradle 			   	# Main Gradle build script
├── 📝 gradle.properties 		  # Build settings and Gradle configs
├── 🖥️ gradlew 				  # Unix Gradle wrapper
├── 🖥️ gradlew.bat 			  # Windows Gradle wrapper
└── ⚙️ settings.gradle 		  # Gradle project/module setup
```


### 🔑 Key Folders
- **TeamCode/** → Our custom programs (autonomous routines, teleop driver code, helper classes).
- **FtcRobotController/** → Unmodified FTC SDK project files.
- **doc/** → Notes on rules, design, electrical/mechanical docs, and team strategies.
- **libs/** → Place for third-party or custom libraries.

---

## 🚦 Features

- 🎮 **Multi-action buttons** – A single button can trigger multiple actions by checking both the `previousGamepad` and `currentGamepad` states.
- 📊 **Telemetry** – Real-time feedback to the Driver Station, including sensor readings, motor powers, and runtime info.
- 🌐 **FTC Dashboard integration** – Provides a web-based interface for live telemetry, tuning, and debugging.
- 🧩 **Modular controllers** – Each subsystem has its own controller class (e.g., `IntakeController`, `RampController`), making code reusable and organized.
- ⚡ **Flexible TeleOp architecture** – Easy to add new routines, map controls, or tweak behavior without changing core code.
- 🛠️ **Hardware abstraction** – `RobotMap` centralizes all hardware definitions, simplifying changes and preventing hardcoded values.
- 🧪 **Safe, testable routines** – Designed for debugging and telemetry logging so behavior can be verified before full deployment.

---

## 🤝 Contributing

Team members should:
1. Work on feature branches (`feature/autonomous`, `fix/drive-train`) and, if authorized, merge to keep the history organized.
2. Follow consistent commit messages.
3. Document significant changes in `doc/`.
4. Open Pull Requests for review before merging into `main`.

---

## 📜 License

This repository is licensed under the **BSD-3-Clause-Clear License**.  
Check the `LICENSE` file for details.

---

## 👥 Team & Acknowledgments

- **Team ID / Name**: FGCVEN25 (Venezuela).
- **Competition**: FIRST Global Challenge 2025.
- **Thanks to**: Mentors, sponsors, and the FIRST and FIRST Global community!

---

> ✨ *"Building robots, building futures."* ✨
