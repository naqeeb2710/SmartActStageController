import sys
import smaract.ctl as ctl
import time
from collections import namedtuple
import numpy as np
from scipy.optimize import minimize
from ctypes import c_double, byref

# Define the SmaractPositioners class
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


        buffer = ctl.FindDevices()

        if len(buffer) == 0:
            print("No MCS2 devices found.")
            print("Exit")
            exit(1)
        buffer = buffer.split("\n")

        mcs = buffer[0]
        self.handle = ctl.Open(mcs)
        serial_num = ctl.GetProperty_s(
            self.handle, 0, ctl.Property.DEVICE_SERIAL_NUMBER
        )
        print(f"Connected to MCS2 with serial number: {serial_num}")
        # 
        # self.handle.F = handle
        no_of_channels = ctl.GetProperty_i32(self.handle, 0, ctl.Property.NUMBER_OF_CHANNELS)
        print("MCS2 number of channels: {}.".format(no_of_channels))
        for channel in range(no_of_channels):
            state = ctl.GetProperty_i32(self.handle, channel, ctl.Property.CHANNEL_STATE)
            # The returned channel state holds a bit field of several state flags.
            # See the MCS2 Programmers Guide for the meaning of all state flags.
            # We pick the "sensorPresent" flag to check if there is a positioner connected
            # which has an integrated sensor.
            # Note that in contrast to previous controller systems the controller supports
            # hotplugging of the sensor module and the actuators.
            if state & ctl.ChannelState.SENSOR_PRESENT:
                print("MCS2 channel {} has a sensor.".format(channel))
            else:
                print("MCS2 channel {} has no sensor.".format(channel))


        move_mode = ctl.MoveMode.CL_ABSOLUTE


        
    # def calibrate(self, positioner_channel_map=None, calibrate_all_channels=True):
    #     """
    #     Calibrates the positioners.

    #     Parameters
    #     ----------
    #     positioner_channel_map : dict
    #         Dictionary with keys representing positioners to calibrate and
    #         values being the specific channels of each positioner to calibrate.
    #         Ignored if calibrate_all_channels set to True.
    #     calibrate_all_channels : bool
    #         If True, calibrates all channels of all positioners.
    #     """

    #     self.isMoving = True

    #     if calibrate_all_channels:
    #         print("Calibrating...")
    #         for positioner in self.handle._fields:
    #             handle = getattr(self.handle, positioner)
    #             for channel in range(getattr(self.numChannels, positioner)):
    #                 ctl.SetProperty_i32(
    #                     handle, channel, ctl.Property.CALIBRATION_OPTIONS, 0
    #                 )
    #                 ctl.Calibrate(handle, channel)

    #     else:
    #         positioners = list(positioner_channel_map.keys())
    #         for positioner in positioners:
    #             print(f"Calibrating positioner: {positioner}.")
    #             handle = getattr(self.handle, positioner)
    #             channels = list(positioner_channel_map[positioner])
    #             for channel in channels:
    #                 print(f"Calibrating channel: {channel}")
    #                 ctl.SetProperty_i32(
    #                     handle, channel, ctl.Property.CALIBRATION_OPTIONS, 0
    #                 )
    #                 ctl.Calibrate(handle, channel)

    #     self.waitForMoving()
        
    
    def calibrate(self):
        """
        Calibrates all channels of the positioners.
        """
        self.isMoving = True

        print("Calibrating...")
        for channel in range(ctl.GetProperty_i32(self.handle, 0, ctl.Property.NUMBER_OF_CHANNELS)):
            ctl.SetProperty_i32(self.handle, channel, ctl.Property.CALIBRATION_OPTIONS, 0)
            ctl.Calibrate(self.handle, channel)
        
        print("Calibrating done")

        # self.waitForMoving()


    # def findReference(self, positioner_channel_map=None, reference_all_channels=True):
    #     """
    #     References the positioners.

    #     Parameters
    #     ----------
    #     positioner_channel_map : dict
    #         Dictionary with keys representing positioners to reference and
    #         values being the specific channels of each positioner to reference.
    #         Ignored if reference_all_channels set to True.
    #     reference_all_channels : bool
    #         If True, all channels of all positioners will be referenced. If
    #         False, only the channels specified in positioner_channel_map will
    #         be referenced.
    #     """

    #     self.isMoving = True

    #     if reference_all_channels:
    #         print("Referencing all channels")
    #         for positioner in self.handle._fields:
    #             handle = getattr(self.handle, positioner)
    #             for channel in range(getattr(self.numChannels, positioner)):
    #                 ctl.SetProperty_i32(
    #                     handle, channel, ctl.Property.REFERENCING_OPTIONS, 0
    #                 )
    #                 ctl.SetProperty_i64(
    #                     handle, channel, ctl.Property.MOVE_VELOCITY, 2_000_000_000
    #                 )
    #                 ctl.SetProperty_i64(
    #                     handle, channel, ctl.Property.MOVE_ACCELERATION, 1_000_000_000
    #                 )
    #                 ctl.Reference(handle, channel)

    #     else:
    #         positioners = list(positioner_channel_map.keys())
    #         for positioner in positioners:
    #             print(f"Referencing positioner: {positioner}.")
    #             handle = getattr(self.handle, positioner)
    #             channels = list(positioner_channel_map[positioner])
    #             for channel in channels:
    #                 print(f"Finding reference on channel: {channel}.")
    #                 ctl.SetProperty_i32(
    #                     handle, channel, ctl.Property.REFERENCING_OPTIONS, 0
    #                 )
    #                 # Set velocity to 2mm/s
    #                 ctl.SetProperty_i64(
    #                     handle, channel, ctl.Property.MOVE_VELOCITY, 2_000_000_000
    #                 )
    #                 # Set acceleration to 1mm/s2.
    #                 ctl.SetProperty_i64(
    #                     handle, channel, ctl.Property.MOVE_ACCELERATION, 1_000_000_000
    #                 )
    #                 # Start referencing sequence
    #                 ctl.Reference(handle, channel)

    #     self.waitForMoving()

    def findReference(self):
        """
        References all channels of the positioners.
        """
        self.isMoving = True

        print("Referencing all channels")
        
        for channel in range(ctl.GetProperty_i32(self.handle, 0, ctl.Property.NUMBER_OF_CHANNELS)):
            ctl.SetProperty_i32(self.handle, channel, ctl.Property.REFERENCING_OPTIONS, 0)
            ctl.SetProperty_i64(self.handle, channel, ctl.Property.MOVE_VELOCITY, 2_000_000_000)
            ctl.SetProperty_i64(self.handle, channel, ctl.Property.MOVE_ACCELERATION, 1_000_000_000)
            ctl.Reference(self.handle, channel)
        
        print("Referencing all channels done")

        self.waitForMoving()

    
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


def main():
    # Instantiate the SmaractPositioners class
    positioners = SmaractPositioners()

    # Calibrate the positioners
    positioners.calibrate()

    # Find reference for the positioners
    positioners.findReference()

    # close 
    positioners.close()

if __name__ == "__main__":
    main()
