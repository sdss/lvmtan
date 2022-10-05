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

from Nice import I_LOG, N_LOG, U7_LOG, A_LOG, F_LOG, E_LOG

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

        self.derot_buffer = 20
        self.derot_dist = 7

    def _status(self, reachable=True):
        return {**BasdaMoccaXCluPythonServiceWorker._status(self), 
                **{"CurrentTime": self.service.getCurrentTime() if reachable else "Unknown",
                   "Simulate": self.simulate}
               }

    async def slewTickSimulate(self, command, delta_time):
        while True:
            try:
                position = math.degrees(self.sid.fieldAngle(self.geoloc, self.point, None))
                U7_LOG(f"field angle {position} deg")
                self.service.moveAbsolute(position, "DEG")


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
            async def setSegment(parent, idx, t0, t1=None):
#                U7_LOG (f"{idx%dbuf} {t0[0]} {t0[1]} {t0[2]} {t0[3]} {t0[4]}")
                await parent._chat(1, 221, parent.device_module, 0, f"{idx%parent.derot_buffer} {t0[0]} {t0[1]} {t0[2]} {t0[3]} {t0[4]}")
                if t1:
#                    U7_LOG(f"{(idx+1)%dbuf} 0 0 {t1[2]} 0 0")
                     await parent._chat(1, 221, parent.device_module, 0, f"{(idx+1)%parent.derot_buffer} 0 0 {t1[2]} 0 0")

            try:
                # clear buffer
                rc = await self._chat(1, 226, self.device_module)

            except Exception as ex:
                pass

            # create buffer
            rc = await self._chat(1, 220, self.device_module, self.derot_buffer)

            now = astropy.time.Time.now()
            traj = self.sid.mpiaMocon(self.geoloc, self.point, None, deltaTime=delta_time, homeOffset=0, polyN=self.derot_dist, time=now)

            N_LOG(f"traj {traj}")

    #        km.moveAbsolute(traj[0][3])

            for i in range(self.derot_dist):
                await setSegment(self, i, traj[i])
            await setSegment(self, i+1, traj[i+1])

            N_LOG(f"last segment {i}")

            # profile start from beginning
            await self._chat(1, 222, self.device_module, 0)


            upidx = self.derot_dist
            while True:
                try:
                    rc = await self._chat(1, 225, self.device_module)
                    moidx = int(rc[0].split(' ')[-1])
                    updistance=((upidx%self.derot_buffer)-moidx+self.derot_buffer)%self.derot_buffer
                    U7_LOG(f"pos: {self.service.getIncrementalEncoderPosition()} {self.service.getDeviceEncoderPosition()}" 
                           f"updist: {updistance} idx: {upidx}")
                    if updistance < self.derot_dist:
                        nowpdt = now + astropy.time.TimeDelta(delta_time*upidx, format='sec')
                        N_LOG(f"{nowpdt} {upidx}")
                        self.sid.mpiaMocon(self.geoloc, self.point, None, deltaTime=delta_time, homeOffset=0, polyN=1, time=nowpdt)

                        await setSegment(self, upidx, traj[0], traj[1])
                        upidx+=1
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
                    await asyncio.sleep(0.2)

                except Exception as ex:
                    E_LOG(ex)
                    break

        except Exception as ex:
            F_LOG(ex)


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
#                await self.slewTickMocon(command, delta_time)


        except Exception as e:
            command.fail(error=e)

        I_LOG(f"done")
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


        if not self.simulate:
            ## profile stop
            rc = await self._chat(1, 224, self.device_module)
            U7_LOG(f"{rc}")

            ## clear buffer
            rc = await self._chat(1, 226, self.device_module)
            U7_LOG(f"{rc}")

        U7_LOG("done")


        return command.finish(**self._status())
        
