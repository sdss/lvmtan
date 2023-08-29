# -*- coding: utf-8 -*-
#
# @Author: Florian Briegel (briegel@mpia.de)
# @Date: 2021-06-15
# @Filename: BasdaMoccaBaseCluPythonServiceWorker.py
# @License: BSD 3-clause (http://www.opensource.org/licenses/BSD-3-Clause)

import asyncio

from sys import maxsize
import time
from random import random
from datetime import datetime
from math import nan

from Basda import ServiceIsBusyException

import BasdaMoccaException
import BasdaMoccaX
import BasdaService
import Nice
from Nice import E_LOG, I_LOG, U9_LOG, A_LOG, F_LOG, N_LOG, W_LOG, U7_LOG

import json

import numpy as np

from .BasdaCluPythonServiceWorker import (BasdaCluPythonServiceWorker, Command,
                                          asyncio, click, command_parser)


# TODO: use cluplus
@command_parser.command(name='__commands')
@click.pass_context
def __commands(ctx, command: Command):
    """Returns all commands."""

    # we have to use the help key for the command list, dont want to change the standard model.
    command.finish(help=[k for k in ctx.command.commands.keys() if k[:2] != '__'])



class BasdaMoccaBaseCluPythonServiceWorker(BasdaCluPythonServiceWorker):
    "python clu worker"

    def __init__(self, _svcName):
        BasdaCluPythonServiceWorker.__init__(self, _svcName)
        self.schema["properties"]["AtLimit"] = {"type": "boolean"}
        self.schema["properties"]["AtHome"] = {"type": "boolean"}
        self.schema["properties"]["Moving"] = {"type": "boolean"}
        self.schema["properties"]["Reachable"] = {"type": "boolean"}
        self.schema["properties"]["CurrentTime"] = {"type": "number"}
        self.schema["properties"]["IncrementalEncoderPosition"] = {"type": "number"}
        self.schema["properties"]["Position"] = {"type": "number"}
        self.schema["properties"]["DeviceEncoder"] = {"Position": {"type": "number"}, "Unit": {"type": "string"}}
        self.schema["properties"]["AbsoluteEncoderPosition"] = {"type": "number"}
        self.schema["properties"]["Velocity"] = {"type": "number"}
        self.schema["properties"]["PositionSwitchStatus"] = {"type": "number"}
        self.schema["properties"]["Simulate"] = {"type": "boolean"}
        self.schema["properties"]["ChatRc"] = {"type": "array"}
        self.schema["properties"]["Site"] = {"type": "string"}

        self.hasLimitSwitch = True
        self.statusCacheData = {}
        self.statusCacheTimestamp = datetime(2000,1,1)
        self.statusCacheAge = 1.0


    async def _stopMovement(self):
        try:
            self.service.stop()
            while self.service.isMoving():
                await asyncio.sleep(0.3)
            return await self._status(self.service.isReachable())

        except Exception as e:
            command.fail(error=e)


    @command_parser.command("stop")
    @BasdaCluPythonServiceWorker.wrapper
    async def stopMovement(self, command: Command):
        """Stop"""
        try:

            return command.finish(**await self._stopMovement())

        except Exception as e:
            command.fail(error=e)


    async def _status(self, reachable=True):
        """Status implementation"""
        lock = asyncio.Lock()
            
        async with lock:
            age = (datetime.now() - self.statusCacheTimestamp).total_seconds()
            if  age > self.statusCacheAge:
                    try:
                        startPollTime = datetime.now()

                        switchStatusName = None
                        switchStatusValue = nan
                        if self.hasLimitSwitch:
                            switchStatusName = "AtLimit"
                            switchStatusValue = self.service.isAtLimit() if reachable else nan
                        else:
                            switchStatusName = "PositionSwitchStatus"
                            switchStatusValue = int(self.service.getPositionSwitchStatus()[0].getValue()) if reachable else nan

                        self.statusCacheData = {
                            "Reachable": reachable,
                            "AtHome": self.service.isAtHome() if reachable else "Unknown",
                            "Moving": self.service.isMoving() if reachable else "Unknown",
                            switchStatusName: switchStatusValue,
                            "Position": self.service.getPosition() if reachable else nan,
                            "DeviceEncoder": {"Position": self.service.getDeviceEncoderPosition("STEPS") if reachable else nan,
                                              "Unit": "STEPS"},
                            "Velocity": self.service.getVelocity() if reachable else nan,
                        }

                        if self.hasIncrementalEncoder:
                            self.statusCacheData["IncrementalEncoderPosition"] = self.service.getIncrementalEncoderPosition() if reachable else nan

                        self.statusCacheTimestamp = datetime.now()
                        polltime = (datetime.now()-startPollTime).total_seconds()
                        if polltime > 1.0:
                            I_LOG(f"status age: {age}, poll time > 1s: {polltime}")

                    except Exception as e:
                        E_LOG(f"Exception: {e}")
                        command.fail(error=e)

        return self.statusCacheData


    @command_parser.command("status")
    @BasdaCluPythonServiceWorker.wrapper
    async def status(self, command: Command):
        """Check status"""
        try:
            return command.finish( ** await self._status(self.service.isReachable()) )

        except Exception as e:
            command.fail(error=e)


    @command_parser.command("isReachable")
    @BasdaCluPythonServiceWorker.wrapper
    async def isReachable(self, command: Command):
        """Check hardware reachability"""
        try:
            return command.finish(
                Reachable=self.service.isReachable()
            )
        except Exception as e:
            command.fail(error=e)


    @command_parser.command("getPositionSwitchStatus")
    @BasdaCluPythonServiceWorker.wrapper
    async def getPositionSwitchStatus(self, command: Command):
        """Returns position switches status"""
        try:
            return command.finish(
                PositionSwitchStatus=self.service.getPositionSwitchStatus()[0].getValue()
            )
        except Exception as e:
            command.fail(error=e)

    @command_parser.command("isAtHome")
    @BasdaCluPythonServiceWorker.wrapper
    async def isAtHome(self, command: Command):
        """Check if at home position"""
        try:
            return command.finish(
                AtHome=self.service.isAtHome()
            )
        except Exception as e:
            command.fail(error=e)

    @command_parser.command("isMoving")
    @BasdaCluPythonServiceWorker.wrapper
    async def isMoving(self, command: Command):
        """Check if moving"""
        try:
            return command.finish(
                Moving=self.service.isMoving()
            )
        except Exception as e:
            command.fail(error=e)


    def _chat(self, card: int, com: int, module: int, select: int=maxsize, params: str="", lines: int=maxsize):
        """Check hardware reachability"""
        if lines == maxsize: lines = ""
        if select == maxsize: select = ""

        try:
            rc = None
#            I_LOG(f" {card} {com} {module} {select} {params} {lines}: call")
            time.sleep(0.01)
            self.service.send(str(card), str(com), str(module), str(select), str(params), str(lines))
            time.sleep(0.02)
            rc = self.service.receive().split('\n')
#            I_LOG(f" {card} {com} {module} {select} {params} {lines}: {rc}")

        except ServiceIsBusyException as ex:
            W_LOG(f"got busy exception - wait and try again {card} {com} {module} {select} {params} {lines} {'r' if rc else 's'}")
            time.sleep(0.2)
            self.service.send(str(card), str(com), str(module), str(select), str(params), str(lines))
            time.sleep(0.02)
            rc = self.service.receive().split('\n')
#            I_LOG(f"+ {card} {com} {module} {select} {params} {lines}: {rc}")

        if int(rc[-1].split(' ')[3]) < 0:
            raise Exception(f"Error #{rc[-1]}")

        return rc


    @command_parser.command("chat")
    @click.argument("CARD", type=int)
    @click.argument("COM", type=int)
    @click.argument("MODULE", type=int)
    @click.argument("SELECT", type=int, default=maxsize)
    @click.argument("PARAMS", type=str, default="")
    @click.argument("LINES", type=int, default=maxsize)
    @BasdaCluPythonServiceWorker.wrapper
    async def chat(self, command: Command, card: int, com: int, module: int, select: int, params: str, lines: int):
        """Check hardware reachability"""
        try:
            rc = self._chat(card, com, module, select, params, lines)
            return command.finish(
                ChatRc = rc
            )

        except Exception as e:
            command.fail(error=e)



