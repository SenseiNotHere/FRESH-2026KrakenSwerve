from wpilib import DriverStation
from wpimath.geometry import Translation2d

from constants.field_constants import Trench

class TrenchAssist:

    @classmethod
    def get_active_centers(cls):
        alliance = DriverStation.getAlliance()

        if Trench.FIELD_TYPE == "Welded":
            if alliance == DriverStation.Alliance.kRed:
                return [
                    Trench.WELDED_RED_TRENCH_BOTTOM_CENTER,
                    Trench.WELDED_RED_TRENCH_TOP_CENTER,
                ]
            else:
                return [
                    Trench.WELDED_BLUE_TRENCH_BOTTOM_CENTER,
                    Trench.WELDED_BLUE_TRENCH_TOP_CENTER,
                ]

        else:  # AndyMark
            if alliance == DriverStation.Alliance.kRed:
                return [
                    Trench.ANDYMARK_RED_TRENCH_BOTTOM_CENTER,
                    Trench.ANDYMARK_RED_TRENCH_TOP_CENTER,
                ]
            else:
                return [
                    Trench.ANDYMARK_BLUE_TRENCH_BOTTOM_CENTER,
                    Trench.ANDYMARK_BLUE_TRENCH_TOP_CENTER,
                ]
