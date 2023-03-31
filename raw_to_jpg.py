
from metavision_core.event_io import EventsIterator
from metavision_sdk_core import OnDemandFrameGenerationAlgorithm

import os
from tqdm import tqdm

# fix args
def parse_args():
    import argparse
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='RAW to JPG.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--input-file', dest='input_path', required=True, help="Path to input RAW file")
    parser.add_argument('-o', '--output-dir', required=True, help="Path to JPGs output directory")
    #parser.add_argument('-s', '--start-ts', type=int, default=0, help="start time in microsecond")
    #parser.add_argument('-d', '--max-duration', type=int, default=1e6 * 60, help="maximum duration in microsecond")
    #parser.add_argument('--delta-t', type=int, default=1000000, help="Duration of served event slice in us.")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()
    
    # FIX - not csv
    if os.path.isfile(args.input_path):
        output_file = os.path.join(args.output_dir, os.path.basename(args.input_path)[:-4] + ".JPG")
    else:
        raise TypeError(f'Fail to access file: {args.input_path}')


    # ? - args
    mv_iterator = EventsIterator(input_path=args.input_path, delta_t=args.delta_t, start_ts=args.start_ts,
                                 max_duration=args.max_duration)
    
    
    # Event Frame Generator
    # Convert .raw file into a series of .jpg to match up w/ the RGB .jpg data
    # External trigger events to mark the start/stop points -> EXT_TRIGGER = 1010
    # OnDemandFrameGenerationAlgorithm - manually control frame generation timestamps

    on_demand_gen = OnDemandFrameGenerationAlgorithm(sensor_width=640, sensor_height=480, accumulation_time_us=50000)

    # For each word in the recording
    for evs in mv_iterator:
        # Check if the event is an EXT_TRIGGER (1010) event
        if evs.type() == 0xA: #sdk.EventType.EXT_TRIGGER
            # Extract the timestamp from the event
            timestamp = evs.t()

            # Generate a frame using the timestamp
            on_demand_gen.generate(timestamp)

            # Get the generated frame and save it as a JPG file
            frame = on_demand_gen.get_latest_frame()
            frame.save_to_jpg(output_file+"frame{}".format(frame.timestamp()))

    # Stop the frame generator
    on_demand_gen.stop()


if __name__ == "__main__":
    main()