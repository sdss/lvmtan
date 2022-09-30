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

        self.derot_buffer = 20
        self.derot_dist = 2

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

        def setSegment(km, idx, t0, t1=None):
            U9_LOG (f"'{idx%cbuf} {t0[0]} {t0[1]} {t0[2]} {t0[3]} {t0[4]}'")
            self._chat(1, 221, module, 0, f"'{idx%cbuf} {t0[0]} {t0[1]} {t0[2]} {t0[3]} {t0[4]}'")
            if t1:
               U9_LOG(f"'{(idx+1)%cbuf} 0 0 {t1[2]} 0 0'")
               self._chat(1, 221, module, 0, f"'{(idx+1)%cbuf} 0 0 {t1[2]} 0 0'")

        try:

            # clear buffer
            rc = self._chat(1, 226, self.device_module)

        except Exception as ex:
            pass

        # create buffer
        rc = self._chat(1, 220, self.device_module, self.derot_buffer)

        now = astropy.time.Time.now()
        traj = self.sid.mpiaMocon(geoloc, point, None, deltaTime=deltaTime, polyN=self.derot_dist, time=now)

#        km.moveAbsolute(traj[0][3])

        for i in range(dist):
            setSegment(self, i, traj[i])
        setSegment(self, i+1, traj[i+1])

        # profile start from beginning
        self._chat(1, 222, module, 0)

        upidx = self.derot_dist
        while True:
            try:
                moidx = int(json.loads(unpack(km.chat(1, 225, module)))[-1].split(' ')[-1])
                updistance=((upidx%cbuf)-moidx+cbuf)%cbuf
                U9_LOG(f"pos: {km.getIncrementalEncoderPosition()} {km.getDeviceEncoderPosition()} updist: {updistance} idx: {upidx}", end = '\n')
                if updistance < dist:
                    nowpdt = now + astropy.time.TimeDelta(deltaTime*upidx, format='sec')
                    U9_LOG(nowpdt)
                    self.sid.mpiaMocon(geoloc, point, None, deltaTime=deltaTime, polyN=1, time=nowpdt)
                    setSegment(km, upidx, traj[0], traj[1])
                    upidx+=1
                sleep(0.2)

            except Exception as ex:
                U9_LOG(ex)
                break


        ### profile stop
        #self._chat(1, 224, module)

        ### clear buffer
        #self._chat(1, 226, module)

        U9_LOG("done")
       #while True:
            #try:
                #position = math.degrees(self.sid.fieldAngle(self.geoloc, self.point, None))
                #U9_LOG(f"field angle {position} deg")

                #command.actor.write(
                     #"i",
                     #{
                        #"Position": self.service.getPosition(),
                        #"DeviceEncoder": {"Position": self.service.getDeviceEncoderPosition("STEPS"), "Unit": "STEPS"},
                        #"Velocity": self.service.getVelocity(),
                        #"AtHome": self.service.isAtHome(),
                        #"AtLimit": self.service.isAtLimit(),
                     #}
                #)

            #except Exception as e:
                 #command.fail(error=e)

            #await asyncio.sleep(delta_time)


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
        
