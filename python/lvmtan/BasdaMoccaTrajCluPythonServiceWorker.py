# -*- coding: utf-8 -*-
#
# @Author: Florian Briegel (briegel@mpia.de)
# @Date: 2021-06-15
# @Filename: BasdaMoccaTrajCluPythonServiceWorker.py
# @License: BSD 3-clause (http://www.opensource.org/licenses/BSD-3-Clause)

import asyncio
import datetime
import math

import astropy.coordinates
import astropy.time
import astropy.units as u
import BasdaMoccaException
import BasdaMoccaTraj
import BasdaService
import Nice
import numpy
import numpy as np
from astropy.utils import iers
from lvmtipo.fiber import Fiber
from lvmtipo.siderostat import Siderostat
from lvmtipo.site import Site
from lvmtipo.target import Target
from Nice import A_LOG, E_LOG, F_LOG, I_LOG, N_LOG, U8_LOG, U1_LOG

from .BasdaMoccaXCluPythonServiceWorker import *
from .exceptions import LvmTanOutOfRange, LvmTanMotorLostSteps


iers.conf.auto_download = False

class BasdaMoccaTrajCluPythonServiceWorker(BasdaMoccaXCluPythonServiceWorker):
    "python clu worker"

    def __init__(self, _svcName):
        BasdaMoccaXCluPythonServiceWorker.__init__(self, _svcName)

        self.schema["properties"]["SkyPA"] = {"type": "number"}
        self.schema["properties"]["LostSteps"] = {"type": "number"}

        self.task = None
        self.task_loop_active = False

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

        if (self.cfgNode.exist("HOME_OFFSET") and self.cfgNode.node("HOME_OFFSET").hasLeaf()):
            self.homeOffset = self.cfgNode.node("HOME_OFFSET").Float
        else:
            self.homeOffset = 135.0


        self.sid = Siderostat(azang=azang, medSign=medSign)

        self.derot_buffer = 10000
        self.seg_time_default = 1
        self.seg_min_num_default = 17
        self.backlashInSteps = 1000
        self.homeIsWest = False

        I_LOG(f"site: {self.site}, homeOffset: {self.homeOffset}, homeIsWest: {self.homeIsWest}, azang: {azang}, medSign {medSign}")

    def _status(self, reachable=True):
        return {**BasdaMoccaXCluPythonServiceWorker._status(self),
                "CurrentTime": self.service.getCurrentTime() if reachable else "Unknown",
                "Simulate": self.simulate,
                "SkyPA": self.service.getDeviceEncoderPosition("SKY"),
               }

    def _sid_mpiaMocon(self, target, polyN=1, seg_time=1, time=None):
        if not time:
             time = astropy.time.Time.now()
        traj = self.sid.mpiaMocon(self.geoloc,
                                  target,
                                  None,
                                  deltaTime=seg_time,
                                  homeIsWest=self.homeIsWest,
                                  homeOffset=self.homeOffset,
                                  polyN=polyN,
                                  time=time)

        U8_LOG(f"{time} {traj}")
        return traj

    async def _slewTickSimulate(self, command, target, seg_time):
        while True:
            try:
                position = self._sid_mpiaMocon(target, seg_time=seg_time)[0][2]

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

            await asyncio.sleep(seg_time)


    async def _slewTickMocon(self, command, target, seg_time:int, seg_min_num:int, offsetInSteps:int):

        try:
            async def setSegment(parent, idx, t0, t1=None, offsetInSteps:int=0):
                cmd = f"{idx%parent.derot_buffer} {t0[0]} {t0[1]} {t0[2]+int(offsetInSteps)}"
                I_LOG (f"{offsetInSteps} {cmd}")
                await parent._chat(1, 221, parent.device_module, 0, cmd)
                if t1:
                    U1_LOG (f"{offsetInSteps} {cmd}")
                    cmd = f"{(idx+1)%parent.derot_buffer} 0 0 {t1[2]+int(offsetInSteps)}"
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
            traj = self._sid_mpiaMocon(target, polyN=seg_min_num, seg_time=seg_time)

#            N_LOG(f"traj {traj}")

            for i in range(seg_min_num + 1):
                await setSegment(self, i, traj[i], offsetInSteps=offsetInSteps)
#            await setSegment(self, i+1, traj[i+1])

#            N_LOG(f"last segment {i}")

            # profile start from beginning
            await self._chat(1, 222, self.device_module, 0)

            upidx = seg_min_num
            while self.task_loop_active:
                try:
                    rc = await self._chat(1, 225, self.device_module)
                    moidx = int(rc[0].split(' ')[-1])
                    updistance=((upidx % self.derot_buffer) - moidx + self.derot_buffer) % self.derot_buffer
                    if updistance < seg_min_num:
                        nowpdt = now + astropy.time.TimeDelta(seg_time * upidx * astropy.units.second)
                        N_LOG(f"{nowpdt} {upidx}")
                        traj = self._sid_mpiaMocon(target, time=nowpdt)

                        try:
                            await setSegment(self, upidx, traj[0], traj[1], offsetInSteps=offsetInSteps)
                            upidx+=1
#                            U8_LOG(f"pos: {self.service.getIncrementalEncoderPosition()} {self.service.getDeviceEncoderPosition()} "
#                                  f"updist: {updistance} idx: {upidx}")
                            
                        except Exception as ex:
                            W_LOG(f"Failed setting segment, try again in {seg_time} updist: {updistance} idx: {upidx}")
                        
                        command.actor.write(
                           "i",
                           {
                                **self._status(self.service.isReachable())
                                #"Position": self.service.getPosition(),
                                #"SkyPA": self.service.getDeviceEncoderPosition("SKY"),
                                #"DeviceEncoder": {"Position": self.service.getDeviceEncoderPosition("STEPS"), "Unit": "STEPS"},
                                #"Velocity": self.service.getVelocity(),
                                #"AtHome": self.service.isAtHome(),
                                #"AtLimit": self.service.isAtLimit(),
                            },
                            internal=True
                        )
                    st =  seg_time
                    while self.task_loop_active and st > 0:
                        st-=1
                        await asyncio.sleep(0.5)

                except Exception as ex:
                    E_LOG(ex)
#                    break

        except Exception as ex:
            F_LOG(ex)

        try:
            # profile stop
            rc = await self._chat(1, 224, self.device_module)

            while self.service.isMoving():
                await asyncio.sleep(0.3)

        except Exception as ex:
            F_LOG(ex)

        I_LOG("Good bye and thanks for alll the fish")


    async def _slewStop(self):
        """Stop slew"""

        if not self.task_loop_active:
            return

        try:
            self.task_loop_active = False
            await asyncio.wait_for(self.task, timeout=5)

        except asyncio.TimeoutError:
            W_LOG("Killing slew task")
            self.task.cancel()
            try:
                await self.task

            except asyncio.CancelledError:
                U7_LOG("slew task is cancelled now")
            
            if not self.simulate:
                ## profile stop
                rc = await self._chat(1, 224, self.device_module)

            while self.service.isMoving():
                await asyncio.sleep(0.3)

            if not self.simulate:
                ## clear buffer
                rc = await self._chat(1, 226, self.device_module)

        self.task = None

        #if self.task:
            #self.task.cancel()
            #try:
                #await self.task

            #except asyncio.CancelledError:
                #U7_LOG("slew task is cancelled now")


            #if not self.simulate:
                ### profile stop
                #rc = await self._chat(1, 224, self.device_module)

            #while self.service.isMoving():
                #await asyncio.sleep(0.3)

            #if not self.simulate:
                ### clear buffer
                #rc = await self._chat(1, 226, self.device_module)

            #self.task = None


    @command_parser.command("slewStart")
    @click.argument("RA", type=float)
    @click.argument("DEC", type=float)
    @click.option("--offset_angle", type=float, default=0.0, show_default=True, help="Derotation offset in degree (9000 motor steps/sky degree).",)
    @click.option("--seg_time", type=int, default=None, help="Time of one trajectory segment in sec.",)
    @click.option("--seg_min_num", type=int, default=None, help="Minimal number of trajectory segments in buffer.",)
    @BasdaCluPythonServiceWorker.wrapper
    async def slewStart(
        self,
        command: Command,
        ra: float,
        dec: float,
        offset_angle: float,
        seg_time: int,
        seg_min_num: int,
    ):
        """Start slew"""
        if not seg_time:
            seg_time = self.seg_time_default
        if not seg_time:
            seg_min_num = self.seg_min_num_default
        
        I_LOG(f"start slew now {ra} {dec} offset_ang: {offset_angle} traj_segment_time: {seg_time} traj_seg_min_num: {seg_min_num} site: {self.site}")

        try:
            await self._slewStop()

            offsetInSteps = offset_angle * 9000.0

            targ = astropy.coordinates.SkyCoord(ra=ra, dec=dec, unit=(u.hourangle, u.deg))
#            I_LOG(astropy.version.version)

            target = Target(targ)

            position = self._sid_mpiaMocon(target)[0][2] + offsetInSteps

            I_LOG(f"field angle {position} steps")
            self.service.moveAbsoluteStart(position - self.backlashInSteps, "STEPS")

            while not self.service.moveAbsoluteCompletion().isDone():
                await asyncio.sleep(0.5)
                #command.info(
                    #Site=self.site,
                    #Position=self.service.getPosition(),
                    #DeviceEncoder={"Position": self.service.getDeviceEncoderPosition("STEPS"), "Unit": "STEPS"},
                    #Velocity=self.service.getVelocity(),
                    #AtHome=self.service.isAtHome(),
                    #AtLimit=self.service.isAtLimit(),
                #)
                command.info( **self._status(self.service.isReachable()) )
            self.service.moveAbsoluteWait()

            self.service.moveRelative(self.backlashInSteps, "STEPS")

            position_error = position - self.service.getDeviceEncoderPosition("STEPS")

            if abs(position_error) > 1:
                A_LOG(f"position error {position_error} steps")
                command.warning(LostSteps=position_error)
                raise LvmTanMotorLostSteps(position_error)



        except Exception as e:
            return command.fail(error=e)

        try:
            loop = asyncio.get_event_loop()

            if self.simulate:
                self.task = loop.create_task(self._slewTickSimulate(command, target, seg_time))
            else:
                self.task = loop.create_task(self._slewTickMocon(command, target, seg_time, seg_min_num, offsetInSteps))
            self.task_loop_active = True

        except Exception as e:
            command.fail(error=e)

        I_LOG(f"done")
        return command.finish(**self._status(self.service.isReachable()))



    @command_parser.command("slewStop")
    @BasdaCluPythonServiceWorker.wrapper
    async def slewStop(
        self,
        command: Command
    ):
        """Stop slew"""
        await self._slewStop()

        U8_LOG("done")

        return command.finish(**self._status(self.service.isReachable()))
