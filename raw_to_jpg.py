
# from metavision_core.event_io import EventsIterator
from metavision_core.event_io import RawReader
from metavision_sdk_core import OnDemandFrameGenerationAlgorithm
import metavision_sdk_core as sdk

import argparse
import os
import time
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
    # mv_iterator = EventsIterator(input_path=args.input_path, delta_t=1, start_ts=0,
                                 # max_duration=10)
    mv_iterator = RawReader(args.input_path)
    # mv_iterator.open()
    
    
    # Event Frame Generator
    # Convert .raw file into a series of .jpg to match up w/ the RGB .jpg data
    # External trigger events to mark the start/stop points -> EXT_TRIGGER = 1010
    # OnDemandFrameGenerationAlgorithm - manually control frame generation timestamps

    on_demand_gen = OnDemandFrameGenerationAlgorithm(width=640, height=480, accumulation_time_us=50000)

    # For each word in the recording

    print(mv_iterator)
    mv_iterator.load_delta_t(1)
    print(mv_iterator)
    events = mv_iterator.get_ext_trigger_events()
    print(events)
    for evs in events:
    # while True:

        # Check if the event is an EXT_TRIGGER (1010) event
        # Extract the timestamp from the event
        timestamp = evs.timestamp()

        # Generate a frame using the timestamp
        on_demand_gen.generate(timestamp)

        # Get the generated frame and save it as a JPG file
        frame = on_demand_gen.get_latest_frame()
        frame.save_to_jpg(output_file+"frame{}".format(frame.timestamp()))



if __name__ == "__main__":
    main()
