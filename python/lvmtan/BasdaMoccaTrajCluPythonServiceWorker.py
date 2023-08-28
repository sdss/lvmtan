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
from .exceptions import *


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
        self.backlashInSteps = 300
        self.homeIsWest = False
        self.skydegToSteps = 9000

        self.seg_time_current = self.seg_time_default = 1
        self.seg_min_num_current = self.seg_min_num_default = 3
        self.seg_offset_add = None

        self.traj_time_left = 0.0
 
        self.homeToIncEncOffset = -140
 
        I_LOG(f"site: {self.site}, homeOffset: {self.homeOffset}, homeIsWest: {self.homeIsWest}, azang: {azang}, medSign {medSign}")




    async def _stopMovement(self):
        try:
            if self.task:
                await self._slewStop()
            else:
                self.service.stop()
                while self.service.isMoving():
                    await asyncio.sleep(0.3)
            
            return await self._status(self.service.isReachable())

        except Exception as e:
            command.fail(error=e)


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

#        N_LOG(f"{time} {traj}")
        return traj



    async def _slewTickMocon(self, command, target, offset_in_steps):

        try:
            async def setSegment(parent, idx, t0, t1=None, offset_in_steps:int=0):
                cmd = f"{idx%parent.derot_buffer} {t0[0]} {t0[1]} {t0[2]+int(offset_in_steps)}"
                U9_LOG (f"{offset_in_steps} {cmd}")
                parent._chat(1, 221, parent.device_module, 0, cmd)
                if t1:
                    cmd = f"{(idx+1)%parent.derot_buffer} 0 0 {t1[2]+int(offset_in_steps)}"
                    U9_LOG (f"{offset_in_steps} {cmd}")
                    parent._chat(1, 221, parent.device_module, 0, cmd)

            def addOffset(seg, offset:int):
                
                cycsteps=seg[0]
                velocity=seg[1]
                current_steps=round(velocity*cycsteps/65536.0)
                velocity=round(65536.0*(current_steps+offset)/cycsteps)

                absposition=seg[2]+offset
                if offset >= 0:
                    return [[cycsteps, velocity, int(absposition)], [0, 0, int(absposition+current_steps)]]
                # else:
                #     return [[0, 0, seg[2]], [cycsteps, velocity, absposition]]

            try:
                # clear buffer
                rc = self._chat(1, 226, self.device_module)

            except Exception as ex:
#                await asyncio.sleep(0.1)
                pass

            # create buffer
            rc = self._chat(1, 220, self.device_module, self.derot_buffer, 0)

            traj = self._sid_mpiaMocon(target, polyN=self.seg_min_num_current, seg_time=self.seg_time_current)

            traj_start = astropy.time.Time.now()
            for i in range(self.seg_min_num_current+1):
                await setSegment(self, i, traj[i], offset_in_steps=offset_in_steps)

            # profile start from beginning
            self._chat(1, 222, self.device_module, 0)
            incencoder=self.service.getIncrementalEncoderPosition() - self.homeToIncEncOffset
            devencoder=self.service.getDeviceEncoderPosition("STEPS")
            I_LOG(f"{self.conn['name']}: incdiff: {incencoder-devencoder}")


            upidx = self.seg_min_num_current
            while self.task_loop_active:
                try:
                    traj_next_seg_time = traj_start + astropy.time.TimeDelta(self.seg_time_current * upidx * astropy.units.second)
                    self.traj_time_left = (traj_next_seg_time - astropy.time.Time.now()).datetime.total_seconds()

                    if self.traj_time_left < self.seg_min_num_current * self.seg_time_current:
                        moidx = None

                        try:
                            rc = self._chat(1, 225, self.device_module)
                            moidx = int(rc[0].split(' ')[-1])

                        except Exception as e:
                            W_LOG(f"handled exception: {e}")
                            pass

#                        N_LOG(f"offCur: {offset_in_steps} offAdd: {self.seg_offset_add}")
                        traj = self._sid_mpiaMocon(target, seg_time=self.seg_time_current, time = traj_next_seg_time)
                        if self.seg_offset_add:
                            traj=addOffset(traj[0], self.seg_offset_add)
                            I_LOG(f"Changed traj: {traj}")
                            await setSegment(self, upidx, traj[0], traj[1], offset_in_steps=offset_in_steps)                            
                            offset_in_steps+=self.seg_offset_add
                            self.seg_offset_add=None
                        else:
                            await setSegment(self, upidx, traj[0], traj[1], offset_in_steps=offset_in_steps)

#                        await setSegment(self, upidx, traj[0], traj[1], offset_in_steps=offset_in_steps)
                        
                        if self.traj_time_left  < (self.seg_min_num_current-1 * self.seg_time_current):
                            N_LOG(f"{self.conn['name']}: tleft: {self.traj_time_left} idx: {upidx} moidx: {moidx}")
#                        I_LOG(f"{self.conn['name']}: tleft: {self.traj_time_left} idx: {upidx} moidx: {moidx}")

                        upidx+=1

                        status = await self._status(self.service.isReachable())
                        devencpos=int(status['DeviceEncoder']['Position'])
                        incencoder=self.service.getIncrementalEncoderPosition() - self.homeToIncEncOffset
                        devencoder=self.service.getDeviceEncoderPosition("STEPS")
#                        I_LOG(f"{self.conn['name']}: incdiff: {incencoder-devencoder} positionInSteps: {devencpos} {traj[0][2]-devencpos} tleft: {self.traj_time_left} idx: {upidx} moidx: {moidx}")
                        
                        command.actor.write("i", **status, internal=True)

                        try:
                            if upidx > self.seg_min_num_current + 3 and not status["Moving"]:
                                F_LOG(f"{self.conn['name']}: Movement stopped - moveToHome() is required !")
                                break

                        except Exception as e:
                            pass

                    await asyncio.sleep(0.3)
                        
                except Exception as ex:
                    W_LOG(f"exception received: {ex}")
                    await asyncio.sleep(0.5)

            incencoder=self.service.getIncrementalEncoderPosition() - self.homeToIncEncOffset
            devencoder=self.service.getDeviceEncoderPosition("STEPS")
            I_LOG(f"{self.conn['name']}: incdiff: {incencoder-devencoder}")


        except Exception as ex:
            F_LOG(ex)
        try:
            # profile stop
            rc = self._chat(1, 224, self.device_module)

            while self.service.isMoving():
                await asyncio.sleep(0.3)

        except Exception as ex:
            F_LOG(ex)

        command.actor.write(
            "i", await self._status(self.service.isReachable()),
                    internal=True
            )

        I_LOG("Good bye and thanks for all the fish")


    async def _slewStop(self):
        """Stop slew"""

        if not self.task:
            return

        try:
            self.task_loop_active = False
            await asyncio.wait_for(self.task, timeout=10)

        except asyncio.TimeoutError:
            W_LOG("Killing slew task")
            self.task.cancel()

            try:
                await self.task

            except asyncio.CancelledError:
                U7_LOG("slew task is cancelled now")
            
            ## profile stop
            rc = self._chat(1, 224, self.device_module)

            #while not self.service.isMoving():
                #await asyncio.sleep(0.3)

            ## clear buffer
            rc = self._chat(1, 226, self.device_module)

        self.task = None

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
        self.seg_time_current = seg_time if seg_time else self.seg_time_default
        self.seg_min_num_current = seg_min_num if seg_min_num else self.seg_min_num_default

        
        I_LOG(f"start slew now {ra} {dec} offset_ang: {offset_angle} traj_segment_time: { self.seg_time_current} traj_seg_min_num: {self.seg_min_num_current} site: {self.site}")

        try:
            await self._slewStop()

            offset_in_steps = offset_angle * self.skydegToSteps

            targ = astropy.coordinates.SkyCoord(ra=ra, dec=dec, unit=(u.hourangle, u.deg))
#            I_LOG(astropy.version.version)

            target = Target(targ)

            position = self._sid_mpiaMocon(target)[0][2] + offset_in_steps

            incencoder = self.service.getIncrementalEncoderPosition() - self.homeToIncEncOffset
            devencoder = self.service.getDeviceEncoderPosition("STEPS")
            diffencoder = devencoder - incencoder
            if abs(diffencoder) > 42:
                W_LOG(f"{self.conn['name']}: adjusting sw encoder with inc encoder, incdiff: {diffencoder} {devencoder} {incencoder}")
                self.service.setPosition(incencoder, "STEPS")

#            I_LOG(f"field angle {position} steps")
            self.service.moveAbsoluteStart(position - self.backlashInSteps, "STEPS")

            while not self.service.moveAbsoluteCompletion().isDone():
                await asyncio.sleep(0.5)
                command.info( **await self._status(self.service.isReachable()) )

            self.service.moveAbsoluteWait()

            self.service.moveRelative(self.backlashInSteps, "STEPS")

            position_error = position - self.service.getDeviceEncoderPosition("STEPS")

            if abs(position_error) > 1:
                A_LOG(f"position error {position_error} steps")
                command.warning(LostSteps=position_error)
                raise LvmTanPositionError(position_error)

        except Exception as e:
            return command.fail(error=e)

        try:
            loop = asyncio.get_event_loop()

            self.task = loop.create_task(self._slewTickMocon(command, target, offset_in_steps))
            self.task_loop_active = True

        except Exception as e:
            return command.fail(error=e)

        for i in range(7):
            await asyncio.sleep(0.1)
            if self.service.isMoving():
                I_LOG(f"start derotating done")
                break

        return command.finish(**await self._status(self.service.isReachable()))


    @command_parser.command("slewStop")
    @BasdaCluPythonServiceWorker.wrapper
    async def slewStop(
        self,
        command: Command
    ):
        """Stop slew"""
        await self._slewStop()

        U8_LOG("done")

        return command.finish(**await self._status(self.service.isReachable()))


    @command_parser.command("slewAdjust")
    @click.option("--offset_angle", type=float, default=0.0, show_default=True, help="Derotation offset in degree (9000 motor steps/sky degree).",)
    @click.option("--seg_min_num", type=int, default=None, help="Minimal number of trajectory segments in buffer.",)
    @BasdaCluPythonServiceWorker.wrapper
    async def slewAdjust(
        self,
        command: Command,
        offset_angle: float,
        seg_min_num: int,
    ):
        """Adjust slew"""

        if offset_angle:
            if self.seg_offset_add:
                return command.fail(error=LvmTanOffsetNotDone())
            if offset_angle < 0:
                return command.fail(error=LvmTanNotImplemented())

            self.seg_offset_add = offset_angle * self.skydegToSteps
            await asyncio.sleep(self.seg_time_current*3)

        if seg_min_num:
            time_to_wait = abs(seg_min_num - self.seg_min_num_current) * self.seg_time_current
            self.seg_min_num_current = seg_min_num
            await asyncio.sleep(time_to_wait)

        return command.finish(**await self._status(self.service.isReachable()))
        
