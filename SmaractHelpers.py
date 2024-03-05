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
        self.initialPositions = {'Px': 971554327, 'Py': -3698276503, 'Pz': 10382713079, 'Nx': 18333363, 'Ny': 67901253, 'Nz': 11006460359, 'Fx': -4061049684, 'Fy': -40666, 'Fz': 21464}
        # self.initialPositions = {'Px': -6616351787, 'Py': -8384844742, 'Pz': -7268522220, 'Nx': -7623147926, 'Ny': 245836393, 'Nz': -7215446352, 'Fx': 154652940, 'Fy': -125583641, 'Fz': -128413939}
        # Center of camera image and about 200um above the surface

        # 1100 microns lateral device separation is 940 pixels
        self.pixelToPositionScaling = 1_100_000_000.0 / 940

        self.zLimit = 600_000_000  # Max amount z can be lowered from initial position
        self.fault = False

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
            "MCS2-00006321": "P",
            "MCS2-00006109": "N",
            "MCS2-00006322": "F",
            # "MCS2-00002553": "F",
            
        }

        self.handle = namedtuple("Handle", "P N F")

        buffer = ctl.FindDevices()

        if len(buffer) == 0:
            print("No MCS2 devices found.")
            print("Exit")
            exit(1)
        buffer = buffer.split("\n")

        if len(buffer) > 3:
            print("Too many MCS2 devices found!")
            print(f"Expected 3 devices, found {len(buffer)}")
            print("Exit")
            exit(1)

        if len(buffer) < 3:
            print(f"Found {len(buffer)} MCS2 devices instead of 3.")
            # connected_devices_serial_nums = []
            # for mcs in buffer:
            #     handle = ctl.Open(mcs)
            #     serial_num = ctl.GetProperty_s(
            #         handle, 0, ctl.Property.DEVICE_SERIAL_NUMBER
            #     )
            #     connected_devices_serial_nums.append(serial_num)
            # missing_serials = set(mcs_serials.keys()).difference(
            #     connected_devices_serial_nums
            # )

            # for missing in missing_serials:
            #     print(
            #         f"MCS2 with serial number {missing} is not connected. Corresponds to {mcs_serials[missing]} positioner"
            #     )

            print("Exit.")
            exit(1)

        else:
            for mcs in buffer:
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
                if assigned_probe == "P":
                    self.handle.P = handle
                elif assigned_probe == "N":
                    self.handle.N = handle
                elif assigned_probe == "F":
                    self.handle.F = handle

        self.numChannels = namedtuple("Channels", "P N F")

        self.numChannels.P = ctl.GetProperty_i32(
            self.handle.P, 0, ctl.Property.NUMBER_OF_CHANNELS
        )
        self.numChannels.N = ctl.GetProperty_i32(
            self.handle.N, 0, ctl.Property.NUMBER_OF_CHANNELS
        )
        self.numChannels.F = ctl.GetProperty_i32(
            self.handle.F, 0, ctl.Property.NUMBER_OF_CHANNELS
        )

        # if not (self.numChannels.P == self.numChannels.N == self.numChannels.F == 3):
        #     print("Not all positioners have 3 axes. Something's wrong. Exit.")
        #     exit(1)

        self.numChannels.P = 3
        self.numChannels.N = 3
        self.numChannels.F = 3

        channel_inversion_dict = {"N": (0, 2), "P": (0, 1, 2), "F": (1,)}

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
                    handle, channel, ctl.Property.MOVE_ACCELERATION, 800_000_000
                )  # 1mm/s^2

        # Converts a positioner axis to a channel number recognised by the MCS2.
        self.axisToChannelDict = {
            "Px": 0,
            "Py": 1,
            "Pz": 2,
            "Nx": 0,
            "Ny": 1,
            "Nz": 2,
            "Fx": 0,
            "Fy": 1,
            "Fz": 2,
        }

        # This is the displacement from initial positions to move to when commanded to move to home.
        self.initialPositionsToHome = {
            "Px": 10_000_000_000,
            "Py": 10_000_000_000,
            "Pz": 10_000_000_000,
            "Nx": 10_000_000_000,
            "Ny": -10_000_000_000,
            "Nz": 10_000_000_000,
            "Fx": -10_000_000_000,
            "Fy": 0,
            "Fz": 10_000_000_000,
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
        """
        Sets a higher velocity and acceleration for the fiber alignment
        procedure so it is faster.
        """
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
        """
        Moves the fiber to the specified position and returns the photocurrent
        reading from the specified instrument.

        Parameters
        ----------
        pos : dict
            Dictionary with keys representing positioners to move and values
            being the absolute position to move each positioner to.

        Returns
        -------
        photocurrent : float
            Photocurrent reading from the specified instrument after moving the
            fiber.
        """
        pos_dict = {"Fx": pos[0], "Fy": pos[1], "Fz": pos[2]}
        self.moveMultiplePositionerChannelsToAbsolutePosition(pos_dict)
        photocurrent = self.getFeedbackValue()
        print(f"\n{-photocurrent*1e6} uA")

        return photocurrent


    def getFeedbackValue(self):
        """
        Returns the photocurrent reading from the specified instrument.
        """

        if self._instType == "sourcemeter":
            return self.feedbackInstrumentHandle.measurePhotocurrent()
        elif self._instType == "TLPM":
            power = c_double()
            self.feedbackInstrumentHandle.measPower(byref(power))
            return power.value
        else:
            print("Unknown feedbackInstrumentHandle type.")


    def optimiseFiber(
        self,
        feedbackInstrumentHandle,
        instType="sourcemeter",
        coarseThresh= -50e-6,
        fineThresh= -700e-6
    ):
        """
        Optimises the fiber position using either a sourcemeter photocurrent
        or a TLPM power reading as the feedback.

        Parameters
        ----------
        feedbackInstrumentHandle : ctl.Instrument, TLPM, or otherwise
            Instrument handle to use as the feedback.
        instType : str
            Type of feedback instrument. Either "sourcemeter" or "TLPM".
        coarseThresh : float, optional
            Coarse threshold of the feedback signal for the optimisation. Below
            this threshold, the optimisation will be done over a larger range.
        fineThresh : float, optional
            Fine threshold of the feedback signal for the optimisation.
            The optimisation will be done until the feedback signal is above
            this threshold.
        """

        start = time.time()
        self._instType = instType
        self.feedbackInstrumentHandle = feedbackInstrumentHandle
        self.setFiberAlignmentVelocity()
        x0_initial, _, _ = self.getFiberPosition(returnType="list")

        attempts = 0
        while self.getFeedbackValue() > fineThresh:
            attempts += 1
            x0, y0, z0 = self.getFiberPosition(returnType="list")
            if self.getFeedbackValue() > coarseThresh:
                yCoarse = 10_000_000
                zCoarse = 10_000_000
                print("Using coarse bounds")
                print(abs(x0_initial - x0))
                if abs(x0_initial - x0) > 70_000_000:
                    print("Exceeded 70um distance for fiber x+, resetting.")
                    xLower = 50_000_00
                    xHigher = 0
                else:
                    xLower = 5_000_000
                    xHigher = 15_000_000
                bounds = [
                    (x0-xLower, x0+xHigher),
                    (y0-yCoarse, y0+yCoarse),
                    (z0-zCoarse, z0+zCoarse)
                    ]

            else:
                print("Using fine bounds")
                bounds = [
                    (x0-2_000_000, x0+4_000_000),
                    (y0-1_000_000, y0+1_000_000),
                    (z0-1_000_000, z0+1_000_000)
                    ]

            xyz0 = np.array([x0, y0, z0])
            minimize(self.fiberFeedback, xyz0, method="Nelder-Mead",
            bounds=bounds, options=self.optimiserOptions)

            if attempts > 7:
                print("Failed to reach target photocurrent after 8 attempts.")
                break

        if self.getFeedbackValue() < fineThresh:
            for _ in range(3):
                print("Doing final fine optimisation...")
                x0, y0, z0 = self.getFiberPosition(returnType="list")
                bounds = [
                    (x0-2_000_000, x0+4_000_000),
                    (y0-1_000_000, y0+1_000_000),
                    (z0-1_000_000, z0+1_000_000)
                    ]

                xyz0 = np.array([x0, y0, z0])
                minimize(self.fiberFeedback, xyz0, method="Nelder-Mead",
                bounds=bounds, options=self.optimiserOptions)

        end = time.time()
        print(f"Time taken to optimise fiber: {end - start:.1f}s")

        self.optimisedFiberResults.append([self.getFiberPosition(), self.getFeedbackValue()])


    def moveToInitialPosition(self):
        """
        Moves the piezos to their initial positions.
        """
    
        print("Moving all positioners to initial position")
        self.moveMultiplePositionerChannelsToAbsolutePosition(self.initialPositions)

    def setDevicePixelDisplacement(self, allDevicesPixelDisplacements):
        """
        Stores the device pixel displacement for all devices.

        Parameters
        ----------
        allDevicesPixelDisplacements : dict
            Dictionary with keys representing devices and values being the
            pixel displacement for each device.
        """

        self.allDevicesPixelDisplacements = allDevicesPixelDisplacements


    def pixelsToPicometers(self, devicePixelDisplacement):
        """
        Converts the pixel displacement of a device to a spatial position in
        picometers.

        Parameters
        ----------
        devicePixelDisplacement : int or float
            Pixel displacement of the devices from the middle of the image.

        Returns
        -------
        probePositionerToPosition : dict
            Dictionary with keys representing channels and values being the
            channel position in picometers for each probe channel.
        fiberPositionerToPosition : dict
            Dictionary with keys representing channels and values being the
            channel position in picometers for each fiber channel.
        """

        # Converting pixels to displacement in picometers
        xAbsoluteDisplacement = round(
            devicePixelDisplacement[0] * self.pixelToPositionScaling
        )
        yAbsoluteDisplacement = round(
            devicePixelDisplacement[1] * self.pixelToPositionScaling
        )

        self.probePositionerToPosition = {}
        self.probePositionerToPosition["Px"] = (
            self.initialPositions["Px"] + xAbsoluteDisplacement
        )
        self.probePositionerToPosition["Py"] = (
            self.initialPositions["Py"] + yAbsoluteDisplacement
        )
        self.probePositionerToPosition["Nx"] = (
            self.initialPositions["Nx"] + xAbsoluteDisplacement
        )
        self.probePositionerToPosition["Ny"] = (
            self.initialPositions["Ny"] + yAbsoluteDisplacement
        )

        # Need to figure out how to define the fiber position.
        self.fiberPositionerToPosition = {}
        self.fiberPositionerToPosition["Fx"] = (
            self.initialPositions["Fx"] + xAbsoluteDisplacement
        )
        self.fiberPositionerToPosition["Fy"] = (
            self.initialPositions["Fy"] + yAbsoluteDisplacement
        )

        return self.probePositionerToPosition, self.fiberPositionerToPosition


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

    def seperate(self,distance):
        for positioner, position in distance.items():
            handle=getattr(self.handle,positioner[0])
            channel=self.axisToChannelDict[positioner]
            position=int(position)
            ctl.Move(handle,channel,position,0)
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
        ----------
        Added conditions on y and z axis movement due to stage crashing into 
        the y-axis piezo.    
        """
        fiber_postion = self.getFiberPosition(returnType="dict")
        fiber_postion.update(axis_position_dict)

        if fiber_postion["Fy"] > 7_130_000_000 and fiber_postion["Fz"] < -2_000_000_000:
            print("Fiber piezo at risk of crashing into itself. Halting process")
            
            self.fault = True
            return

        self.isMoving = True
        for positioner, position in axis_position_dict.items():
            handle = getattr(self.handle, positioner[0])
            channel = self.axisToChannelDict[positioner]
            position = int(position)
            ctl.Move(handle, channel, position, 0)

        self.waitForMoving()
    
    def moveBy(self, axis_position_dict):
        """
        Moves multiple positioner channels by a position specified in
        picometers.

        Parameters
        ----------
        axis_position_dict : dict
            Dictionary with keys representing positioner axis and values being
            the absolute position to move to in picometers.
            e.g. {"Px": 5_000_000_000, "Ny": 3_000_000_000} to move Px by 5mm
            and Ny by 3mm.
        """
        fiber_postion = self.getFiberPosition(returnType="dict")
        fiber_postion.update(axis_position_dict)

        if fiber_postion["Fy"] > 7_130_000_000 and fiber_postion["Fz"] < -2_000_000_000:
            print("Fiber piezo at risk of crashing into itself. Halting process")
            
            self.fault = True
            return

        self.isMoving = True
        for positioner, position in axis_position_dict.items():
            handle = getattr(self.handle, positioner[0])
            channel = self.axisToChannelDict[positioner]
            position = int(position) + ctl.GetProperty_i64(handle, channel, ctl.Property.POSITION)
            ctl.Move(handle, channel, position, 0)

        self.waitForMoving()


    def moveProbesToDevice(self, deviceID, Motor=None):
        """
        Moves the probes to the device position.

        Parameters
        ----------
        deviceID : int
            Device ID to move to.
        Motor : Motor handle, optional
            Handle for motorised rotation stages for ND filters. If given,
            the ND filters will be rotated to minimum transmission for imaging
            and forward bias purposes.
        """
        if Motor is not None:
            Motor.move_to_position({"A": 50, "B": 20})
        self.pixelsToPicometers(self.allDevicesPixelDisplacements[deviceID])
        print(f"Moving to device {deviceID}")
        self.moveMultiplePositionerChannelsToAbsolutePosition(
            self.probePositionerToPosition
        )

        # self.allDevicesPixelDisplacements = None          # Set this to None after using it once so we are forced to take another image before we can move again


    def moveFiberToNextDevice(self):
        """
        Moves the fiber to a device's edge coupler.

        Parameters
        ----------
        deviceID : int
            Device ID to move to.
        """

        x, y, z = self.getFiberPosition(returnType="list")

        x -= 50_000_000
        y += 275_000_000    # 275um is the waveguide spacing

        pos_dict = {
        "Fx": x,
        "Fy": y,
        "Fz": z
        }
        print("Moving fiber to next device")
        self.moveMultiplePositionerChannelsToAbsolutePosition(pos_dict)
        # self.allDevicesPixelDisplacements = None          # Set this to None after using it once so we are forced to take another image before we can move again

    def getFiberPosition(self, returnType="dict"):
        """
        Gets the current absolute position of the fiber.

        Parameters
        ----------
        returnType : str, optional
            Type of return value. Options are "dict" (default) or "list".
            "dict" returns a dictionary with keys representing positioner
            axis and values being the position in picometers.
            "list" returns a list of the position in picometers ordered as
            [Fx, Fy, Fz].
        
        Returns
        -------
        position : dict or list
        """
        
        positions = self.getPositions()
        if returnType == "dict":
            return {
                "Fx": positions["Fx"],
                "Fy": positions["Fy"],
                "Fz": positions["Fz"]
            }
        elif returnType == "list":
            return [positions["Fx"], positions["Fy"], positions["Fz"]]
        else:
            raise ValueError("returnType must be either 'dict' or 'list'")
    

    def retractToHome(self):
        """
        Retracts the positioners to the home position.
        """

        dict_to_move = {}
        for positioner, position in self.initialPositionsToHome.items():
            dict_to_move[positioner] = self.initialPositions[positioner] + position

        print("Retracting probes")
        self.moveMultiplePositionerChannelsToAbsolutePosition(dict_to_move)


    def moveOutOfCamera(self, Motor=None):
        """
        Moves the positioner out of the camera. If a ND filter motor is
        given, the ND filter will be rotated to minimum transmission for
        imaging purposes.
        """

        print("Moving probes out of camera view")

        dict_to_move = {
            "Py": self.initialPositions["Py"] + 3_000_000_000,
            "Ny": self.initialPositions["Ny"] - 3_000_000_000,
        }
        if Motor is not None:
            Motor.move_to_position({"A": 0})
        self.moveMultiplePositionerChannelsToAbsolutePosition(dict_to_move)


    def waitForMoving(self):
        """
        Waits for the positioners to finish moving.
        """

        while True:
            moving = []
            for positioner in ["P", "N", "F"]:
                handle = getattr(self.handle, positioner)
                for channel in range(3):
                    status = ctl.GetProperty_i32(
                        handle, channel, ctl.Property.CHANNEL_STATE
                    )
                    moving.append((status & ctl.ChannelState.ACTIVELY_MOVING) != 0)

            time.sleep(0.01)
            if not any(moving):
                break
        self.isMoving = False

        return self.isMoving


    def raiseProbes(self, raiseBy=500_000_000):
        """
        Raises the probes to a safe height for moving them
        around. Default is 500um relative to the initial
        position. Can be changed by passing a value to
        raiseBy.

        Parameters
        ----------
        raiseBy : int or float
            Amount to raise the probes by in picometers.
        """

        print("Raising probes")
        dict_to_move = {
            "Pz": self.initialPositions["Pz"] + raiseBy,
            "Nz": self.initialPositions["Nz"] + raiseBy,
        }

        self.moveMultiplePositionerChannelsToAbsolutePosition(dict_to_move)
        self.fault = False  # Raising probably cleared the 'fault'

    def raiseNProbe(self, raiseBy=500_000_000):
        #raise N probe
        print("Raising N probe")
        dict_to_move = {
            "Nz": self.initialPositions["Nz"] + raiseBy
        }
        self.moveMultiplePositionerChannelsToAbsolutePosition(dict_to_move)
        self.fault = False

    def raisePProbe(self, raiseBy=500_000_000):
        #raise N probe
        print("Raising P probe")
        dict_to_move = {
            "Pz": self.initialPositions["Pz"] + raiseBy
        }
        self.moveMultiplePositionerChannelsToAbsolutePosition(dict_to_move)
        self.fault = False



    def lowerProbes(self, lowerBy=200_000_000):
        """
        Lowers the probes from a safe height to closer to the devices. Default
        is 200um relative to the initial position. Can be changed by passing a
        value to lowerBy. Does not check if the probes will exceed the zLimit.
        Use incrementallyLower for that.

        Parameters
        ----------
        lowerBy : int or float
            Amount to lower the probes by in picometers.
        """

        print("Lowering probes")
        dict_to_move = {
            "Pz": self.initialPositions["Pz"] - lowerBy,
            "Nz": self.initialPositions["Nz"] - lowerBy,
        }

        self.moveMultiplePositionerChannelsToAbsolutePosition(dict_to_move)


    def incrementallyLower(self, step=5_000_000):
        """
        Lowers the probes incrementally. Default step is 5 microns. Can be
        changed by passing a value to step. This function checks if the probes
        will exceed the zLimit. If they do, it will stop lowering.

        Parameters
        ----------
        step : int or float
            The step size in picometers.
        """

        self.getPositions()
        for positioner in ["Pz", "Nz"]:
            if (
                self.positions[positioner] - step
                < self.initialPositions[positioner] - self.zLimit
            ):
                print(
                    f"Lowering {positioner} would exceed the zLimit. Not lowering probes."

                )
                self.fault = True

                break

        else:
            print("Incrementally lowering probes")
            dict_to_move = {
                "Pz": self.positions["Pz"] - step,
                "Nz": self.positions["Nz"] - step,
            }

            self.moveMultiplePositionerChannelsToAbsolutePosition(dict_to_move)
        
    def incrementallyraise(self, step=5_000_000):
        self.getPositions()
        print("Incrementally raising probes")
        dict_to_move = {
             "Pz": self.positions["Pz"] + step,
             "Nz": self.positions["Nz"] + step,
        }
        self.moveMultiplePositionerChannelsToAbsolutePosition(dict_to_move)



    def getPositions(self):
        """
        Gets the current position of all the positioners.
        """

        self.positions = {}
        for positioner in ["P", "N", "F"]:
            handle = getattr(self.handle, positioner)
            for axis in ["x", "y", "z"]:
                positioner_axis = f"{positioner}{axis}"
                channel = self.axisToChannelDict[positioner_axis]
                position = ctl.GetProperty_i64(handle, channel, ctl.Property.POSITION)
                self.positions[positioner_axis] = position

        return self.positions


    def encoderLightsOff(self):
        """
        Turns off the encoder lights.
        """

        for positioner in self.handle._fields:
            handle = getattr(self.handle, positioner)
            for channel in range(getattr(self.numChannels, positioner)):
                ctl.SetProperty_i32(handle, channel, ctl.Property.SENSOR_POWER_MODE, 0)
        time.sleep(1)
        print("Encoder lights off")

    def encoderLightsOn(self):
        """
        Turns on the encoder lights. A SETTLING TIME IS REQUIRED, otherwise
        the probes WILL SOMETIMES GET STUCK in some undefined state and will
        not move.
        """

        for positioner in self.handle._fields:
            handle = getattr(self.handle, positioner)
            for channel in range(getattr(self.numChannels, positioner)):
                ctl.SetProperty_i32(handle, channel, ctl.Property.SENSOR_POWER_MODE, 1)
        time.sleep(1)  # Settling time. THIS IS IMPORTANT.
        print("Encoder lights on")


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
