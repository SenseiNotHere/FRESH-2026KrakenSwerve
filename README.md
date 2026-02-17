# FRESH 2026 - Kraken Swerve

FRC Team 1811’s 2026 competition robot codebase.

WE'RE STILL COMING UP WITH A ROBOT NAME :P

Fully state-driven.  
Fully superstructured.  
Fully locked in.

This season we refined the control structure to make the robot more intentional and scalable.

---

# 🧠 Architecture Philosophy

We don’t bind buttons straight to motors anymore.

We define intent.

States like `PREP_SHOT`, `INTAKING`, `APPROACHING_TOWER` describe what the robot is trying to accomplish. The superstructure then coordinates drivetrain, shooter, intake, and everything else accordingly.

Clean. Predictable. Scalable.

Teleop and auto share logic.  
Subsystems don’t fight each other.  
And we’re not duct-taping commands together mid-event anymore.

We’re building like we plan to keep winning with it.

---

# ⚙ Hardware Overview

## 🔄 Drivetrain – Kraken Swerve

Located in `subsystems/drive/`

- Drive Motors: Kraken X60  
- Steer Motors: Kraken X44  
- Holonomic swerve drive  
- Field-relative control  
- Vision-assisted alignment  
- Integrated scoring states  

Yes, it’s fast.  
Yes, it tracks.  
Yes, it behaves.

---

## 🎯 Shooter System

Located in `subsystems/shooter/`

- Shooter Motor: Kraken X60  
- Closed-loop velocity control  
- State-gated firing  
- Coordinated indexing  
- Vision-aware prep  

We don’t just press shoot and pray.  
The robot confirms it’s ready.

---

## 📦 Indexer

- Motor: Brushed motor via SparkMax  
- Controlled by superstructure  
- Only feeds when shooter conditions are met  

No panic feeding. No chaos cycling.

---

## 🟢 Intake

- Motor: Brushed motor via SparkMax  
- Pneumatic deployment system  
- State-aware deployment logic  
- Auto-integrated  

It deploys when it should.  
It retracts when it should.  
Ground intake energy handled.

---

## 🧗 Climber

- Pneumatic airbrake system  
- Elevator coordination  
- Manual + autonomous states  

We like controlled climbs.  
No last-second “hope this works” moments.

---

## 👁 Vision System

Located in `subsystems/vision/`

- Limelight  
- AprilTag localization  
- Drivetrain-following states  
- Target-aware scoring logic  

Vision is baked into decision-making.  
It is NOT a side feature.

---

# 🤖 Autonomous

- PathPlanner integration (`deploy/pathplanner/`)  
- Shared superstructure logic  
- Autonomous-specific states  
- Behavior-based sequencing  

Auto runs on the same brain as teleop.

Consistency >>> copy-paste command stacks.

---

# 🛠 Tech Stack

- Python 3.11+  
- RobotPy + WPILib  
- Phoenix 6  
- REV SparkMax  
- PathPlanner  
- Limelight  

Yes, we use Phoenix Pro.  
Yes, we know what we’re doing.

---

# ⚠ IMPORTANT WARNINGS (PLEASE READ BEFORE YOU FAFO)

## 🔧 Robot-Specific Constants

**ALL CONSTANTS IN THIS REPOSITORY ARE ROBOT-SPECIFIC.**

Every value in `constants/` is tuned for:

- Our gear ratios  
- Our inversions  
- Our encoder offsets  
- Our CAN IDs  
- Our robot dimensions  
- Our pneumatics  

If you copy this without changing constants… respectfully… that’s wild.

You MUST:

- Verify CAN IDs  
- Verify inversions  
- Verify encoder offsets  
- Retune all PID values  
- Confirm dimensions  

Failure to update constants may result in:

- Mechanism damage  
- Uncontrolled motion  
- Incorrect field positioning  
- Serious injury  

Yes. Injury.

Misconfigured control loops can cause sudden, high-speed robot movement.

By using this repository, you accept full responsibility for safe implementation.  
FRC Team 1811 is not liable for damage, injury, or misuse.

This is not a plug-and-play template.  
This is architecture.

---

## 🔐 Phoenix 6 Pro Requirement

This robot uses CTRE Phoenix 6 Pro features.

Without:

- A valid Phoenix Pro license  
- Correct Phoenix installation  
- Matching firmware  

Some motors and advanced control modes will not behave properly.

Set up your CTRE stack correctly before deploying.

We are not debugging your licensing.

---

# 🧪 Development Setup

## 1. Create a virtual environment

    python -m venv .venv
    .\.venv\Scripts\Activate.ps1

## 2. Install dependencies

    pip install -r requirements.txt

## 3. Sync dependencies

    robotpy sync

## 4. Run in simulator

    robotpy sim

## 5. DEPLOY AND ENJOY!!

    robotpy deploy --skip-tests

---

# ▶ Running / Simulation

- Use RobotPy simulation tools  
- Or deploy to a roboRIO  

Follow RobotPy + WPILib documentation for deployment instructions.

---

# 📁 Project Structure

    robot.py
    robotcontainer.py
    commands/
    subsystems/
    constants/
    superstructure/
    deploy/
    tests/
    utils/

---

# 🎵 Orchestra

CHRPs are in:

    deploy/files

Yes, the robot can sing.  
Yes, it likely will be Ariana Grande.  
Yes, that’s intentional.

---

# 🤝 How to Contribute

Welcome.

If you're contributing, that means you care about building something clean. We appreciate that.

This robot runs on a state-driven architecture. Respect the structure.

## 🧠 Before You Add Code

- Do not bypass the superstructure.
- Do not bind buttons directly to motor outputs.
- Do not hardcode constants outside `constants/`.
- If you’re unsure where something belongs: ask.

We build systems here. Not shortcuts.

---

## 🗂 Organization Rules

- `subsystems/` -> Hardware logic  
- `commands/` -> Behaviors  
- `superstructure/` -> Robot intent coordination  
- `constants/` -> Tunable values  
- `utils/` -> Helper tools  

If it feels like a hack, it probably is.

---

## 🧪 Test First

- Run simulation before deploying.
- Verify motor directions after mechanical changes.
- Confirm Phoenix Pro configuration before enabling.

The robot moves fast.  
Mistakes move faster.

---

Keep it readable.  
Keep it intentional.  
Keep it elite.

---

Architecture first.  
Results follow.

Built with love by FRC Team 1811 - FRESH.  
2026 is different.
