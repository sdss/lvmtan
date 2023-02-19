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

from Nice import I_LOG, N_LOG, U8_LOG, A_LOG, F_LOG, E_LOG

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

        self.schema["properties"]["SkyPA"] = {"type": "number"}

        target = None
        if (self.rootNode.exist("SITE") and self.rootNode.node("SITE").hasLeaf()):
            self.site = self.rootNode.node("SITE").String
        else:
            self.site = "LCO"

        self.geoloc = Site(name = self.site)

        # but south-> north at LCO
        if self.geoloc.lat > 40.0:
            azang = 180 # MPIA
        else:
            azang = 0  # LCO, APO, KHU

        medSign = -1

        self.sid = Siderostat(azang=azang, medSign=medSign)

        self.derot_buffer = 100
        self.derot_dist = 7
        self.backlashInSteps = 1000
        self.homeOffset = 135.0
        self.homeIsWest = False

        I_LOG(f"site: {self.site}, homeOffset: {self.homeOffset}, homeIsWest: {self.homeIsWest}, azang: {azang}, medSign {medSign}")

    def _status(self, reachable=True):
        return {**BasdaMoccaXCluPythonServiceWorker._status(self), 
                "CurrentTime": self.service.getCurrentTime() if reachable else "Unknown",
                "Simulate": self.simulate,
                "SkyPA": self.service.getDeviceEncoderPosition("SKY"),
               }

    def _sid_mpiaMocon(self, target, polyN=1, delta_time=1, time=None):
        if not time:
             time = astropy.time.Time.now()
        traj = self.sid.mpiaMocon(self.geoloc,
                                  target,
                                  None,
                                  deltaTime=delta_time,
                                  homeIsWest=self.homeIsWest,
                                  homeOffset=self.homeOffset,
                                  polyN=polyN,
                                  time=time)

        U8_LOG(f"{time} {traj}")
        return traj

    async def _slewTickSimulate(self, command, target, delta_time):
        while True:
            try:
                position = self._sid_mpiaMocon(target, delta_time=delta_time)[0][2]

                U8_LOG(f"field angle {position} steps")
                self.service.moveAbsolute(position, "STEPS")

                command.actor.write(
                     "i", 
                     { 
                        "Position": self.service.getPosition(),
                        "SkyPA": self.service.getDeviceEncoderPosition("SKY"),
                        "DeviceEncoder": {"Position": self.service.getDeviceEncoderPosition("STEPS"), "Unit": "STEPS"},
                        "Velocity": self.service.getVelocity(),
                        "AtHome": self.service.isAtHome(),
                        "AtLimit": self.service.isAtLimit(),
                        "Simulate": self.simulate,
                     }
                )
                     
            except Exception as e:
                 E_LOG(f"{e}")
                 command.fail(error=e)

            await asyncio.sleep(delta_time)


    async def _slewTickMocon(self, command, target, delta_time):
        
        try:
            async def setSegment(parent, idx, t0, t1=None):
                cmd = f"{idx%parent.derot_buffer} {t0[0]} {t0[1]} {t0[2]}"
                U8_LOG (cmd)
                await parent._chat(1, 221, parent.device_module, 0, cmd)
                if t1:
                    cmd = f"{(idx+1)%parent.derot_buffer} 0 0 {t1[2]}"
                    U8_LOG(cmd)
                    await parent._chat(1, 221, parent.device_module, 0, cmd)

            try:
                # clear buffer
                rc = await self._chat(1, 226, self.device_module)

            except Exception as ex:
                pass

            # create buffer
#            rc = await self._chat(1, 220, self.device_module, self.derot_buffer)
            rc = await self._chat(1, 220, self.device_module, self.derot_buffer, 0)

            now = astropy.time.Time.now()
            traj = self._sid_mpiaMocon(target, polyN=self.derot_dist, delta_time=delta_time)

#            N_LOG(f"traj {traj}")

            for i in range(self.derot_dist + 1):
                await setSegment(self, i, traj[i])
#            await setSegment(self, i+1, traj[i+1])

#            N_LOG(f"last segment {i}")

            # profile start from beginning
            await self._chat(1, 222, self.device_module, 0)

            upidx = self.derot_dist
            while True:
                try:
                    rc = await self._chat(1, 225, self.device_module)
                    moidx = int(rc[0].split(' ')[-1])
                    updistance=((upidx % self.derot_buffer) - moidx + self.derot_buffer) % self.derot_buffer
                    if updistance < self.derot_dist:
                        nowpdt = now + astropy.time.TimeDelta(delta_time * upidx * astropy.units.second)
#                        N_LOG(f"{nowpdt} {upidx}")
                        traj = self._sid_mpiaMocon(target, time=nowpdt)

                        await setSegment(self, upidx, traj[0], traj[1])

 #                       U8_LOG(f"pos: {self.service.getIncrementalEncoderPosition()} {self.service.getDeviceEncoderPosition()} "
 #                             f"updist: {updistance} idx: {upidx}")

                        upidx+=1
                        command.actor.write(
                            "i",
                            {
                                "Position": self.service.getPosition(),
                                "SkyPA": self.service.getDeviceEncoderPosition("SKY"),
                                "DeviceEncoder": {"Position": self.service.getDeviceEncoderPosition("STEPS"), "Unit": "STEPS"},
                                "Velocity": self.service.getVelocity(),
                                "AtHome": self.service.isAtHome(),
                                "AtLimit": self.service.isAtLimit(),
                            }
                        )
                    await asyncio.sleep(0.3)

                except Exception as ex:
                    E_LOG(ex)
                    break

        except Exception as ex:
            F_LOG(ex)

    async def _slewStop(self):
        """Stop slew"""

        if self.task:
            self.task.cancel()
            try:
                await self.task

            except asyncio.CancelledError:
                U7_LOG("slew task is cancelled now")


            if not self.simulate:
                ## profile stop
                rc = await self._chat(1, 224, self.device_module)

                ## clear buffer
                rc = await self._chat(1, 226, self.device_module)

            while self.service.isMoving():
                await asyncio.sleep(0.02)

            self.task = None



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

        try:
            await self._slewStop()

            targ = astropy.coordinates.SkyCoord(ra=ra, dec=dec, unit=(u.hourangle, u.deg))
#            I_LOG(astropy.version.version)

            target = Target(targ)

            # calculate the field angle (in radians)
            position = self._sid_mpiaMocon(target)[0][2] - self.backlashInSteps


            I_LOG(f"field angle {position} steps")
            self.service.moveAbsoluteStart(position, "STEPS")

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

            if abs(position - self.service.getDeviceEncoderPosition("STEPS")) > 1000:
               A_LOG(f"diff angle {abs(position - self.service.getDeviceEncoderPosition('STEPS')) > 1.0 } steps")
               raise LvmTanOutOfRange()

            self.service.moveRelative(self.backlashInSteps, "STEPS")


        except Exception as e:
            return command.fail(error=e)

        try:
            loop = asyncio.get_event_loop()

            if self.simulate:
                self.task = loop.create_task(self._slewTickSimulate(command, target, delta_time))
            else:
                self.task = loop.create_task(self._slewTickMocon(command, target, delta_time))


        except Exception as e:
            command.fail(error=e)

        I_LOG(f"done")
        return command.finish(** (await self._status(self.service.isReachable())))
            
            

    @command_parser.command("slewStop")
    @BasdaCluPythonServiceWorker.wrapper
    async def slewStop(
        self,
        command: Command
    ):
        """Stop slew"""

        await self._slewStop()

        U8_LOG("done")

        return command.finish(** (await self._status(self.service.isReachable())))
        
