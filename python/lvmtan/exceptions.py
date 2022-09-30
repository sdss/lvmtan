# -*- coding: utf-8 -*-
#
# Licensed under a 3-clause BSD license.
#
# @Author: Florian Briegel
# @Date:   2022-09-05 12:01:21


class LvmTanError(Exception):
    """A custom core Lvmtan exception"""

class LvmTanNotImplemented(LvmTanError):
    """A custom exception for not yet implemented features."""

class LvmTanApiAuthError(LvmTanError):
    """A custom exception for API authentication errors"""


