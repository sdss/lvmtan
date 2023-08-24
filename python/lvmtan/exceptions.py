# -*- coding: utf-8 -*-
#
# Licensed under a 3-clause BSD license.
#
# @Author: Florian Briegel
# @Date:   2022-09-05 12:01:21


class LvmTanError(Exception):
    """Core exception"""

class LvmTanNotImplemented(LvmTanError):
    """Not yet implemented features."""

class LvmTanApiAuthError(LvmTanError):
    """API authentication errors"""

class LvmTanOutOfRange(LvmTanError):
    """Out of range error"""

class LvmTanMotorIsStillMoving(LvmTanError):
    """Motor is still in movement"""

class LvmTanMotorLostSteps(LvmTanError):
    """Motor lost steps"""

class LvmTanPositionError(LvmTanError):
    """Motor wrong position, moveToHome() required"""

class LvmTanOffsetNotDone(LvmTanError):
    """An offset is still applied"""
