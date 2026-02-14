from typing import Optional

from commands2 import Subsystem
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkLowLevel,
    SparkBase,
    ResetMode,
    PersistMode
)
from wpilib import (
    DoubleSolenoid,
    PneumaticsModuleType,
    SmartDashboard,
    SendableChooser
)

from constants.constants import IntakeConstants


class Intake(Subsystem):

    def __init__(
        self,
        motorCANID: int,
        motorInverted: bool,
        solenoidModuleID: int,
        pneumaticsModuleType: PneumaticsModuleType,
        forwardChannel: int,
        reverseChannel: int
    ):
        super().__init__()

        # Motor

        self.motor = SparkMax(
            motorCANID,
            SparkLowLevel.MotorType.kBrushless
        )

        config = SparkMaxConfig()
        config.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
        config.inverted(motorInverted)

        # Closed Loop
        config.closedLoop.P(IntakeConstants.kP)
        config.closedLoop.I(0.0)
        config.closedLoop.D(IntakeConstants.kD)
        config.closedLoop.velocityFF(IntakeConstants.kFF)
        config.closedLoop.outputRange(-1.0, 1.0)

        self.motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self.pid = self.motor.getClosedLoopController()

        # Pneumatics (Deploy)

        self.deploySolenoid = DoubleSolenoid(
            module=solenoidModuleID,
            moduleType=pneumaticsModuleType,
            forwardChannel=forwardChannel,
            reverseChannel=reverseChannel
        )

        self.deployed = False
        self.retract()  # Start safe

        # State

        self._targetRPM: Optional[float] = None

        # Speed chooser
        self.speedChooser = SendableChooser()
        self.speedChooser.setDefaultOption("100%", 1.0)
        self.speedChooser.addOption("75%", 0.75)
        self.speedChooser.addOption("50%", 0.5)
        self.speedChooser.addOption("25%", 0.25)
        self.speedChooser.addOption("0%", 0.0)

        SmartDashboard.putData("Intake Speed", self.speedChooser)

    # Periodic

    def periodic(self):

        if self._targetRPM is None:
            self.motor.set(0.0)
        else:
            self.pid.setReference(
                self._targetRPM,
                SparkBase.ControlType.kVelocity
            )

        SmartDashboard.putBoolean("Intake/Deployed", self.deployed)
        SmartDashboard.putNumber(
            "Intake/Target RPM",
            self._targetRPM if self._targetRPM else 0.0
        )

    # Motor Control

    def intake(self):
        scale = self.speedChooser.getSelected()
        self._targetRPM = IntakeConstants.kIntakeRPS * 60.0 * scale

    def reverse(self):
        scale = self.speedChooser.getSelected()
        self._targetRPM = -IntakeConstants.kIntakeRPS * 60.0 * scale

    def stop(self):
        self._targetRPM = None
        self.motor.set(0.0)

    # Pneumatics

    def deploy(self):
        self.deploySolenoid.set(DoubleSolenoid.Value.kForward)
        self.deployed = True

    def retract(self):
        self.deploySolenoid.set(DoubleSolenoid.Value.kReverse)
        self.deployed = False

    def toggleDeploy(self):
        if self.deployed:
            self.retract()
        else:
            self.deploy()
