"""
Code attempting to synchronize the event-based camera with a frame-based camera.
"""

from metavision_core.event_io.raw_reader import initiate_device
from metavision_core.event_io import EventsIterator, LiveReplayEventsIterator
from metavision_sdk_core import PeriodicFrameGenerationAlgorithm, ColorPalette
from metavision_sdk_ui import EventLoop, BaseWindow, MTWindow, UIAction, UIKeyEvent
import argparse

#test
def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='Metavision camera synchronization sample.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-s', '--serial-number', dest='input_path', default="",
        help="Camera serial number. If not specified, the live stream of the first available camera is used.")

    args = parser.parse_args()
    return args


def main():
    """ Main """
    args = parse_args()

    # Creation of Prophesee HAL device
    device = initiate_device(path=args.input_path)
    #Initiate trigger in
    i_trigger_in = device.get_i_trigger_in()
    i_trigger_in.enable(0); #evk3's trigger in is on channel 0
    #Will stream data as normal, but inject "external event" data into the event stream.
    #Todo: write a Python script to try to decode this data. 
    #See the OnDemandFrameGenerationAlgorithm class - this may be helpful in reconstructing these events into frames.

    #Start the recording
    if device.get_i_events_stream():
        log_path = "recording_" + time.strftime("%y%m%d_%H%M%S", time.localtime()) + ".raw"
        if args.output_dir != "":
            log_path = os.path.join(args.output_dir, log_path)
        print(f'Recording to {log_path}')
        device.get_i_events_stream().log_raw_data(log_path)

    # Events iterator on Device
    mv_iterator = EventsIterator.from_device(device=device)
    height, width = mv_iterator.get_size()  # Camera Geometry

    # Window - Graphical User Interface
    with MTWindow(title="Metavision Events Viewer", width=width, height=height,
                  mode=BaseWindow.RenderMode.BGR) as window:
        def keyboard_cb(key, scancode, action, mods):
            if key == UIKeyEvent.KEY_ESCAPE or key == UIKeyEvent.KEY_Q:
                window.set_close_flag()

        window.set_keyboard_callback(keyboard_cb)

        # Event Frame Generator
        event_frame_gen = PeriodicFrameGenerationAlgorithm(sensor_width=width, sensor_height=height, fps=25, palette=ColorPalette.Dark)

        def on_cd_frame_cb(ts, cd_frame):
            window.show_async(cd_frame)

        event_frame_gen.set_output_callback(on_cd_frame_cb)

        # Process events
        for evs in mv_iterator:
            # Dispatch system events to the window
            EventLoop.poll_and_dispatch()
            event_frame_gen.process_events(evs)

            if window.should_close():
                # Stop the recording
                device.get_i_events_stream().stop_log_raw_data()
                break


if __name__ == "__main__":
    main()
