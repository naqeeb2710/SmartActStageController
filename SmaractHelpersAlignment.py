import sys
import smaract.ctl as ctl
import time
from collections import namedtuple
import numpy as np
from scipy.optimize import minimize
from ctypes import c_double, byref


# TODOS:
# Replace self.waitForMoving with asyncio instead of time.sleep and while True loop
# Use logging instead of print
# Raise exceptions rather than using the fault flag
# Change user-facing functions to use microns rather than picometers, or have an arg to use picometers
# Change namedtuple to dictionaries
# Remove overkill checks in __init__
# Figure out how to define the fiber position
# Fix calibrate and reference functions to use axis names rather than channel numbers
# Use click instead of argparse for command line arguments
# Fix retractToHome function to use global xyz correctly
# Pass the sourcemeter object to this class so the fiber alignment and whatever else needs the sourcemeter can be streamlined into this one class and the automated_measurements script can be much cleaner.


class SmaractPositioners:
    def __init__(self):
        self.ctl = ctl
        self.isMoving = False

        vapi = ctl.api_version
        vlib = [int(i) for i in ctl.GetFullVersionString().split(".")]
        if vapi[0] != vlib[0]:
            raise RuntimeError(
                "Incompatible SmarActCTL python api and library version."
            )

        # This ensures that the same physical
        # device is always assigned to P probe,
        # N probe or fiber positioners
        mcs_serials = {
            "MCS2-00002553": "F",
        }

        self.handle = namedtuple("Handle", "F")

        buffer = ctl.FindDevices()

        if len(buffer) == 0:
            print("No MCS2 devices found.")
            print("Exit")
            exit(1)
        buffer = buffer.split("\n")

        mcs = buffer[0]
        handle = ctl.Open(mcs)
        serial_num = ctl.GetProperty_s(
            handle, 0, ctl.Property.DEVICE_SERIAL_NUMBER
        )
        try:
            assigned_probe = mcs_serials[serial_num]
        except KeyError:
            print("MCS serial numbers have changed?")
            print("Known serial numbers are: ")
            print(*mcs_serials.keys(), sep="\n")
            print(f"However, we got unknown serial number {serial_num}. Exit.")
            exit(1)
        
        self.handle.F = handle
        self.numChannels = namedtuple("Channels", "F")
        self.numChannels.F = 6

        channel_inversion_dict = {"F": (0, 1, 3, 5, 4)}

        for positioner, channels in channel_inversion_dict.items():
            handle = getattr(self.handle, positioner)
            for channel in channels:
                ctl.SetProperty_i32(
                    handle, channel, ctl.Property.LOGICAL_SCALE_INVERSION, 1
                )

        move_mode = ctl.MoveMode.CL_ABSOLUTE
        for positioner in self.handle._fields:
            handle = getattr(self.handle, positioner)
            for channel in range(getattr(self.numChannels, positioner)):

                # Turns on the encoder lights
                ctl.SetProperty_i32(handle, channel, ctl.Property.SENSOR_POWER_MODE, 1)
                ctl.SetProperty_i32(
                    handle, channel, ctl.Property.MAX_CL_FREQUENCY, 500
                )
                # Turn off PID after 1 second
                ctl.SetProperty_i32(
                    handle, channel, ctl.Property.HOLD_TIME, 1_000
                )
                ctl.SetProperty_i32(handle, channel, ctl.Property.MOVE_MODE, move_mode)
                # Set move velocity [in pm/s].
                ctl.SetProperty_i64(
                    handle, channel, ctl.Property.MOVE_VELOCITY, 2_000_000_000
                )  # 2mm/s
                # Set move acceleration [in pm/s2].
                ctl.SetProperty_i64(
                    handle, channel, ctl.Property.MOVE_ACCELERATION, 1_000_000_000
                )  # 1mm/s^2

        # Converts a positioner axis to a channel number recognised by the MCS2.
        self.axisToChannelDict = {
            "Fx": 1,
            "Fy": 2,
            "Fz": 0,
            "Fa": 4,
            "Fb": 3,
            "Fc": 5,
        }

        # Options for scipy minimize
        self.optimiserOptions = {
            "xatol": 50_000,    # +-50nm
            "fatol": 3.0e-6,    # +-3uA
            "disp": True,
            "maxiter": 100,
            "maxfev": 100
        }

        # Save the position of the fiber after optimisation
        self.optimisedFiberResults = []

    def calibrate(self, positioner_channel_map=None, calibrate_all_channels=True):
        """
        Calibrates the positioners.

        Parameters
        ----------
        positioner_channel_map : dict
            Dictionary with keys representing positioners to calibrate and
            values being the specific channels of each positioner to calibrate.
            Ignored if calibrate_all_channels set to True.
        calibrate_all_channels : bool
            If True, calibrates all channels of all positioners.
        """

        self.isMoving = True

        if calibrate_all_channels:
            print("Calibrating...")
            for positioner in self.handle._fields:
                handle = getattr(self.handle, positioner)
                for channel in range(getattr(self.numChannels, positioner)):
                    ctl.SetProperty_i32(
                        handle, channel, ctl.Property.CALIBRATION_OPTIONS, 0
                    )
                    ctl.Calibrate(handle, channel)

        else:
            positioners = list(positioner_channel_map.keys())
            for positioner in positioners:
                print(f"Calibrating positioner: {positioner}.")
                handle = getattr(self.handle, positioner)
                channels = list(positioner_channel_map[positioner])
                for channel in channels:
                    print(f"Calibrating channel: {channel}")
                    ctl.SetProperty_i32(
                        handle, channel, ctl.Property.CALIBRATION_OPTIONS, 0
                    )
                    ctl.Calibrate(handle, channel)

        self.waitForMoving()

    def findReference(self, positioner_channel_map=None, reference_all_channels=True):
        """
        References the positioners.

        Parameters
        ----------
        positioner_channel_map : dict
            Dictionary with keys representing positioners to reference and
            values being the specific channels of each positioner to reference.
            Ignored if reference_all_channels set to True.
        reference_all_channels : bool
            If True, all channels of all positioners will be referenced. If
            False, only the channels specified in positioner_channel_map will
            be referenced.
        """

        self.isMoving = True

        if reference_all_channels:
            print("Referencing all channels")
            for positioner in self.handle._fields:
                handle = getattr(self.handle, positioner)
                for channel in range(getattr(self.numChannels, positioner)):
                    ctl.SetProperty_i32(
                        handle, channel, ctl.Property.REFERENCING_OPTIONS, 0
                    )
                    ctl.SetProperty_i64(
                        handle, channel, ctl.Property.MOVE_VELOCITY, 2_000_000_000
                    )
                    ctl.SetProperty_i64(
                        handle, channel, ctl.Property.MOVE_ACCELERATION, 1_000_000_000
                    )
                    ctl.Reference(handle, channel)

        else:
            positioners = list(positioner_channel_map.keys())
            for positioner in positioners:
                print(f"Referencing positioner: {positioner}.")
                handle = getattr(self.handle, positioner)
                channels = list(positioner_channel_map[positioner])
                for channel in channels:
                    print(f"Finding reference on channel: {channel}.")
                    ctl.SetProperty_i32(
                        handle, channel, ctl.Property.REFERENCING_OPTIONS, 0
                    )
                    # Set velocity to 2mm/s
                    ctl.SetProperty_i64(
                        handle, channel, ctl.Property.MOVE_VELOCITY, 2_000_000_000
                    )
                    # Set acceleration to 1mm/s2.
                    ctl.SetProperty_i64(
                        handle, channel, ctl.Property.MOVE_ACCELERATION, 1_000_000_000
                    )
                    # Start referencing sequence
                    ctl.Reference(handle, channel)

        self.waitForMoving()

    def setFiberAlignmentVelocity(self):
        positioner = "F"
        handle = getattr(self.handle, positioner)
        for channel in range(getattr(self.numChannels, positioner)):
            ctl.SetProperty_i64(
                handle, channel, ctl.Property.MOVE_VELOCITY, 50_000_000
            )  # 100um/s
            # Set move acceleration [in pm/s2].
            ctl.SetProperty_i64(
                handle, channel, ctl.Property.MOVE_ACCELERATION, 100_000_000
            )  # 100um/s^2

    def fiberFeedback(self, pos):
        pos_dict = {"Fx": pos[0], "Fy": pos[1], "Fz": pos[2], "Fa": pos[3], "Fb": pos[4], "Fc": pos[5]}
        self.moveMultiplePositionerChannelsToAbsolutePosition(pos_dict)
        photocurrent = self.getFeedbackValue()
        print()
        print(f"{photocurrent*1e6} uA")
        return -photocurrent

    def getFeedbackValue(self):
        if self._instType == "sourcemeter":
            return self.feedbackInstrumentHandle.measurePhotocurrent()
        elif self._instType == "TLPM":
            power = c_double()
            self.feedbackInstrumentHandle.measPower(byref(power))
            return power.value
        else:
            print("Unknown feedbackInstrumentHandle type.")

    def optimiseFiber(self, feedbackInstrumentHandle, instType="sourcemeter", coarseThresh=50e-6, fineThresh=700e-6):
        start = time.time()
        self._instType = instType
        self.feedbackInstrumentHandle = feedbackInstrumentHandle
        self.setFiberAlignmentVelocity()
        x0_initial, _, _, a0_initial, _, _ = self.getFiberPosition(returnType="list")

        attempts = 0
        while self.getFeedbackValue() < fineThresh:
            attempts += 1
            x0, y0, z0, a0, b0, c0 = self.getFiberPosition(returnType="list")
            if self.getFeedbackValue() < coarseThresh:
                yCoarse = 10_000_000
                zCoarse = 10_000_000
                print("Using coarse bounds")

                if abs(x0_initial - x0) > 60_000_000:
                    print("Exceeded 60um distance for fiber x+, resetting.")
                    xLower = 50_000_00
                    xHigher = 0
                else:
                    xLower = 5_000_000
                    xHigher = 5_000_000

                if abs(a0_initial - a0) > 60_000_000:
                    print("Exceeded 60um distance for fiber a-, resetting.")
                    aLower = 0
                    aHigher = 50_000_00
                else:
                    aLower = 5_000_000
                    aHigher = 5_000_000

                bounds = [
                    (x0-xLower, x0+xHigher),
                    (y0-yCoarse, y0+yCoarse),
                    (z0-zCoarse, z0+zCoarse),
                    (a0-aLower, a0+aHigher),
                    (b0-yCoarse, b0+yCoarse),
                    (c0-zCoarse, c0+zCoarse),
                    ]

            else:
                print("Using fine bounds")
                bounds = [
                    (x0-2_000_000, x0+4_000_000),
                    (y0-1_000_000, y0+1_000_000),
                    (z0-1_000_000, z0+1_000_000),
                    (a0-4_000_000, a0+2_000_000),
                    (b0-1_000_000, b0+1_000_000),
                    (c0-1_000_000, c0+1_000_000),
                    ]

            xyzabc0 = np.array([x0, y0, z0, a0, b0, c0])
            minimize(self.fiberFeedback, xyzabc0, method="Nelder-Mead",
            bounds=bounds, options=self.optimiserOptions)

            if attempts > 10:
                print("Failed to reach target photocurrent after 8 attempts.")
                break

        if self.getFeedbackValue() > fineThresh:
            for _ in range(3):
                print("Doing final fine optimisation...")
                x0, y0, z0, a0, b0, c0 = self.getFiberPosition(returnType="list")
                bounds = [
                    (x0-2_000_000, x0+4_000_000),
                    (y0-500_000, y0+500_000),
                    (z0-500_000, z0+500_000),
                    (a0-4_000_000, a0+2_000_000),
                    (b0-500_000, b0+500_000),
                    (c0-500_000, c0+500_000),
                    ]

                xyzabc0 = np.array([x0, y0, z0, a0, b0, c0])
                minimize(self.fiberFeedback, xyzabc0, method="Nelder-Mead",
                bounds=bounds, options=self.optimiserOptions)

        end = time.time()
        print(f"Time taken to optimise fiber: {end - start:.1f}s")

        self.optimisedFiberResults.append([self.getFiberPosition(), self.getFeedbackValue()])

    def movePositionerChannelToAbsolutePosition(self, positioner, position):
        """
        Moves a positioner channel to an absolute position specified in
        picometers.

        Parameters
        ----------
        positioner : str
            Positioner-axis to move e.g. "Px" or "Nz".
        position : int or float
            Absolute position to move to in picometers.
        """

        self.isMoving = True
        handle = getattr(self.handle, positioner[0])
        channel = self.axisToChannelDict[positioner]
        position = int(position)
        ctl.Move(handle, channel, position, 0)

        self.waitForMoving()

    def moveMultiplePositionerChannelsToAbsolutePosition(self, axis_position_dict):
        """
        Moves multiple positioner channels to an absolute position specified in
        picometers.

        Parameters
        ----------
        axis_position_dict : dict
            Dictionary with keys representing positioner axis and values being
            the absolute position to move to in picometers.
            e.g. {"Px": 5_000_000_000, "Ny": 3_000_000_000} to move Px to 5mm
            and Ny to 3mm.
        """

        self.isMoving = True
        for positioner, position in axis_position_dict.items():
            handle = getattr(self.handle, positioner[0])
            channel = self.axisToChannelDict[positioner]
            position = int(position)
            ctl.Move(handle, channel, position, 0)

        self.waitForMoving()
    
    


    def moveFiberToNextDevice(self):
        """
        Moves the fiber to a device's edge coupler.

        Parameters
        ----------
        deviceID : int
            Device ID to move to.
        """

        x, y, z, a, b, c = self.getFiberPosition(returnType="list")

        x -= 50_000_000
        a += 50_000_000
        y += 275_000_000    # 275um is the waveguide spacing
        b += 275_000_000

        pos_dict = {
        "Fx": x,
        "Fy": y,
        "Fz": z,
        "Fa": a,
        "Fb": b,
        "Fc": c,
        }
        print("Moving fiber to next device")
        self.moveMultiplePositionerChannelsToAbsolutePosition(pos_dict)
        # self.allDevicesPixelDisplacements = None          # Set this to None after using it once so we are forced to take another image before we can move again

    def getFiberPosition(self, returnType="dict"):
        
        positions = self.getPositions()
        if returnType == "dict":
            return {
                "Fx": positions["Fx"],
                "Fy": positions["Fy"],
                "Fz": positions["Fz"],
                "Fa": positions["Fa"],
                "Fb": positions["Fb"],
                "Fc": positions["Fc"],
            }
        elif returnType == "list":
            return [positions["Fx"], positions["Fy"], positions["Fz"], positions["Fa"], positions["Fb"], positions["Fc"]]
        else:
            raise ValueError("returnType must be either 'dict' or 'list'")
    
    def waitForMoving(self):
        """
        Waits for the positioner to finish moving.
        """

        while True:
            moving = []
            for positioner in ["F"]:
                handle = getattr(self.handle, positioner)
                for channel in range(6):
                    status = ctl.GetProperty_i32(
                        handle, channel, ctl.Property.CHANNEL_STATE
                    )
                    moving.append((status & ctl.ChannelState.ACTIVELY_MOVING) != 0)

            time.sleep(0.01)
            if not any(moving):
                break
        self.isMoving = False

        return self.isMoving

    def getPositions(self):
        """
        Gets the current position of all the positioners.
        """

        self.positions = {}
        for positioner in ["F"]:
            handle = getattr(self.handle, positioner)
            for axis in ["x", "y", "z", "a", "b", "c"]:
                positioner_axis = f"{positioner}{axis}"
                channel = self.axisToChannelDict[positioner_axis]
                position = ctl.GetProperty_i64(handle, channel, ctl.Property.POSITION)
                self.positions[positioner_axis] = position

        return self.positions

    def stop(self):
        """
        Stops the probes and the PID control loop.
        """

        for positioner in self.handle._fields:
            handle = getattr(self.handle, positioner)
            for channel in range(getattr(self.numChannels, positioner)):
                ctl.Stop(handle, channel)

        print("Stopping all channels")

    def close(self):
        """
        Closes the connection to the MCS2.
        """

        for positioner in self.handle._fields:
            handle = getattr(self.handle, positioner)
            ctl.Close(handle)

        print("Closed all handles")
