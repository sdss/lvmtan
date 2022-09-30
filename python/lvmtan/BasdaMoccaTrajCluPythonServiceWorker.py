# -*- coding: utf-8 -*-
#
# @Author: Florian Briegel (briegel@mpia.de)
# @Date: 2021-06-15
# @Filename: BasdaMoccaTrajCluPythonServiceWorker.py
# @License: BSD 3-clause (http://www.opensource.org/licenses/BSD-3-Clause)

import datetime

import BasdaMoccaException
import BasdaMoccaTraj
import BasdaService
import Nice
import numpy as np

from Nice import I_LOG, U9_LOG, A_LOG, F_LOG

from .BasdaMoccaXCluPythonServiceWorker import *
from .exceptions import LvmTanOutOfRange

import asyncio
import math
import numpy
import astropy.coordinates
import astropy.time
import astropy.units as u

from lvmtipo.site import Site
from lvmtipo.siderostat import Siderostat
from lvmtipo.fiber import Fiber
from lvmtipo.target import Target

from astropy.utils import iers
iers.conf.auto_download = False

class BasdaMoccaTrajCluPythonServiceWorker(BasdaMoccaXCluPythonServiceWorker):
    "python clu worker"

    def __init__(self, _svcName):
        BasdaMoccaXCluPythonServiceWorker.__init__(self, _svcName)
        self.task = None
        self.geoloc = None
        self.sid = Siderostat()
        self.point = None
        if (
            self.rootNode.exist("SITE") and self.rootNode.node("SITE").hasLeaf()
        ):
            self.site = self.rootNode.node("SITE").String
        else:
            self.site = "LCO"

        self.geoloc = Site(name = self.site)

        I_LOG(f"site: {self.site}")
        I_LOG(f"simulate: {self.simulate}")

    def _status(self, reachable=True):
        return {**BasdaMoccaXCluPythonServiceWorker._status(self), 
                **{"CurrentTime": self.service.getCurrentTime() if reachable else "Unknown",
                   "Simulate": self.simulate}
               }

    async def slewTickSimulate(self, command, delta_time):
        while True:
            try:
                position = math.degrees(self.sid.fieldAngle(self.geoloc, self.point, None))
                U9_LOG(f"field angle {position} deg")
                if self.simulate:
                    self.service.moveAbsolute(position, "DEG")
                else:
                    I_LOG(f"move {position}")


                command.actor.write(
                     "i", 
                     { 
                        "Position": self.service.getPosition(),
                        "DeviceEncoder": {"Position": self.service.getDeviceEncoderPosition("STEPS"), "Unit": "STEPS"},
                        "Velocity": self.service.getVelocity(),
                        "AtHome": self.service.isAtHome(),
                        "AtLimit": self.service.isAtLimit(),
                        "Simulate": self.simulate,
                     }
                )
                     
            except Exception as e:
                 command.fail(error=e)

            await asyncio.sleep(delta_time)

    async def slewTickMocon(self, command, delta_time):

        try:

            # clear buffer
            rc = self._chat(1, 226, self.device_module)
            N_LOG(rc)

        except Exception as ex:
            A_LOG(ex)

        while True:
            try:
                position = math.degrees(self.sid.fieldAngle(self.geoloc, self.point, None))
                U9_LOG(f"field angle {position} deg")

                command.actor.write(
                     "i",
                     {
                        "Position": self.service.getPosition(),
                        "DeviceEncoder": {"Position": self.service.getDeviceEncoderPosition("STEPS"), "Unit": "STEPS"},
                        "Velocity": self.service.getVelocity(),
                        "AtHome": self.service.isAtHome(),
                        "AtLimit": self.service.isAtLimit(),
                     }
                )

            except Exception as e:
                 command.fail(error=e)

            await asyncio.sleep(delta_time)


    @command_parser.command("slewStart")
    @click.argument("RA", type=float)
    @click.argument("DEC", type=float)
    @click.argument("DELTA_TIME", type=int, default=1)
    @BasdaCluPythonServiceWorker.wrapper
    async def slewStart(
        self,
        command: Command,
        ra: float,
        dec: float,
        delta_time: int
    ):
        """Start slew"""
        I_LOG(f"start slew now {ra} {dec} {delta_time} {self.site}")

        targ = astropy.coordinates.SkyCoord(ra=ra, dec=dec, unit=(u.hourangle, u.deg))
#        I_LOG(astropy.version.version)

        self.point = Target(targ)

        # calculate the field angle (in radians)
        try:
            position = math.degrees(self.sid.fieldAngle(self.geoloc, self.point, None))
            
            I_LOG(f"field angle {position} deg")
            self.service.moveAbsoluteStart(position, "DEG")
            while not self.service.moveAbsoluteCompletion().isDone():
                await asyncio.sleep(0.1)
                command.info(
                    Site=self.site,
                    Position=self.service.getPosition(),
                    DeviceEncoder={"Position": self.service.getDeviceEncoderPosition("STEPS"), "Unit": "STEPS"},
                    Velocity=self.service.getVelocity(),
                    AtHome=self.service.isAtHome(),
                    AtLimit=self.service.isAtLimit(),
                )
            self.service.moveAbsoluteWait()

            if abs(position - self.service.getDeviceEncoderPosition("DEG")) > 1.0:
               A_LOG(f"diff angle {abs(position - self.service.getDeviceEncoderPosition('DEG')) > 1.0 } deg")
               raise LvmTanOutOfRange()

        except Exception as e:
            return command.fail(error=e)

        try:
            loop = asyncio.get_event_loop()
            if self.task:
                self.task.cancel()
            if self.simulate:
                self.task = loop.create_task(self.slewTickSimulate(command, delta_time))
            else:
                self.task = loop.create_task(self.slewTickMocon(command, delta_time))

        except Exception as e:
            command.fail(error=e)

        return command.finish(**self._status())
            
            

    @command_parser.command("slewStop")
    @BasdaCluPythonServiceWorker.wrapper
    async def slewStop(
        self,
        command: Command
    ):
        """Stop slew"""
        if self.task:
            self.task.cancel()
            self.task = None

        return command.finish(**self._status())
        
