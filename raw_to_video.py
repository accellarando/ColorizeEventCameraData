#!/usr/bin/python

# Copyright (c) Prophesee S.A. - All Rights Reserved
#
# Subject to Prophesee Metavision Licensing Terms and Conditions ("License T&C's").
# You may not use this file except in compliance with these License T&C's.
# A copy of these License T&C's is located in the "licensing" folder accompanying this file.

"""
This script generates an AVI video from a RAW file
"""

from os import path

from metavision_hal import DeviceDiscovery
from metavision_designer_engine import Controller, KeyboardEvent
from metavision_designer_core import HalDeviceInterface, CdProducer, FrameGenerator, VideoWriter


def parse_args():
    import argparse
    """Defines and parses input arguments"""

    description = "Simple sample on how to generate an AVI video from a RAW file, " + \
        "using the Metavision Designer Python API."

    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('-i', '--input-raw-file', dest='input_filename', metavar='INPUT_FILENAME', required=True,
                        help='Path to input RAW file.')
    parser.add_argument('-o', '--output-video-file', dest='output_video_path', metavar='OUTPUT_VIDEO_PATH',
                        help='Path to output AVI file. If not provided, the base name of the input file will be used.')

    return parser.parse_args()


def main():
    """Main function"""
    args = parse_args()

    # Check provided input file exists
    if not(path.exists(args.input_filename) and path.isfile(args.input_filename)):
        print("Error: provided input path '{}' does not exist or is not a file.".format(args.input_filename))
        return 1

    if args.output_video_path:
        output_video = args.output_video_path
    else:
        output_video, _ = path.splitext(args.input_filename)
        output_video += '.avi'

    # Open file
    device = DeviceDiscovery.open_raw_file(args.input_filename)
    if not device:
        print("Error: could not open file '{}'.".format(args.input_filename))
        return 1

    # Start the streaming of events
    i_events_stream = device.get_i_events_stream()
    i_events_stream.start()

    # Create the controller
    controller = Controller()

    # Create HalDeviceInterface, poll camera buffer every millisecond
    hal_device_interface = HalDeviceInterface(device)
    controller.add_device_interface(hal_device_interface)

    # Create the filtering chains
    # Read CD events from the camera
    prod_cd = CdProducer(hal_device_interface)
    controller.add_component(prod_cd)

    # Generate a graphical representation of the events
    frame_generator = FrameGenerator(prod_cd)
    frame_generator.set_name("CD FrameGenerator")
    controller.add_component(frame_generator)

    # Add Video Writer filter
    video_writer = VideoWriter(frame_generator, output_video, fps=50.)
    controller.add_component(video_writer)
    video_writer.enable(True)

    # Setup rendering with 50 frames per second
    controller.add_renderer(video_writer, Controller.RenderingMode.SimulationClock, 50.)
    controller.enable_rendering(True)

    # Set controller parameters for running :
    controller.set_slice_duration(10000)
    controller.set_batch_duration(100000)

    # Main program loop
    print("Generating file {}".format(output_video))
    while not (controller.is_done()):

        # Get the last key pressed
        last_key = controller.get_last_key_pressed()

        # Exit program if requested
        if last_key == ord('q') or last_key == KeyboardEvent.Symbol.Escape:
            break

        # Run
        controller.run(False)


if __name__ == '__main__':
    import sys
    sys.exit(main())
