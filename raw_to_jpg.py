
from metavision_core.event_io import EventsIterator
from my_raw_reader import RawReader
# from metavision_core.event_io import RawReader
from metavision_sdk_core import OnDemandFrameGenerationAlgorithm
import metavision_sdk_core as sdk

import argparse
import os
import time
from tqdm import tqdm
from PIL import Image

import numpy as np

counter = 0

# fix args
def parse_args():
    import argparse
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='RAW to JPG.',
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--input-file', dest='input_path', required=True, help="Path to input RAW file")
    parser.add_argument('-o', '--output-dir', required=True, help="Path to JPGs output directory")
    parser.add_argument('-r', '--rgb-dir', required=True, help="Path to RGB camera files")
    #parser.add_argument('-s', '--start-ts', type=int, default=0, help="start time in microsecond")
    #parser.add_argument('-d', '--max-duration', type=int, default=1e6 * 60, help="maximum duration in microsecond")
    #parser.add_argument('--delta-t', type=int, default=1000000, help="Duration of served event slice in us.")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    if os.path.isfile(args.input_path):
        output_file = os.path.join(args.output_dir, os.path.basename(args.input_path)[:-4])
    else:
        raise TypeError(f'Fail to access file: {args.input_path}')


    # ? - args
    events_iterator = EventsIterator(input_path=args.input_path, delta_t=1000000, start_ts=0,
                                 max_duration=1e6*60)
    mv_iterator = RawReader(record_base=args.input_path)
    # mv_iterator.open()


    # Event Frame Generator
    # Convert .raw file into a series of .jpg to match up w/ the RGB .jpg data
    # External trigger events to mark the start/stop points -> EXT_TRIGGER = 1010
    # OnDemandFrameGenerationAlgorithm - manually control frame generation timestamps

    on_demand_gen = OnDemandFrameGenerationAlgorithm(width=640, height=480, accumulation_time_us=100000)

    # frame_period_us = 25000
    # next_processing_ts = frame_period_us
    frame = np.zeros((480, 640, 3), np.uint8)
    for evs in events_iterator:
        on_demand_gen.process_events(evs)  # Feed events to the frame generator
        '''
        ts = evs["t"][-1] # Trigger new frame generations as long as the last event is high enough
        while(ts > next_processing_ts):
            on_demand_gen.generate(next_processing_ts, frame)
            next_processing_ts += frame_period_us
        '''

    # For each image from the RGB camera
    imgNames = os.listdir(args.rgb_dir)
    delays = []
    imgs = 0
    for name in imgNames:
        imgParts = name.split("-")
        imgNo = int(imgParts[0][3:])
        if imgNo != imgs:
            delays.append(-1)
        delays.append(imgParts[1].split(".")[0])
        imgs += 1

    # For each word in the recording
    counter = 0
    while not mv_iterator.is_done():
        mv_iterator.load_n_events(10000) 
        events = mv_iterator.get_ext_trigger_events()
        for evs in events:
            imgDelay = delays[counter]
            if imgDelay == -1:
                counter += 1
                continue
            # counter += 1
            # print(evs)
            timestamp = evs[1]
            print(timestamp)
            print(timestamp+int(imgDelay))
            print("\n")

            # Generate a frame using the timestamp plus us differential from the rgb image
            on_demand_gen.generate(timestamp+int(imgDelay),frame)
            # print(frame)

            # Get the generated frame and save it as a JPG file
            # frame.save_to_jpg(output_file+"frame{}".format(frame.timestamp()))
            im = Image.fromarray(frame)
            im.save(output_file+"-{}-frame{}.jpg".format(counter,timestamp));
            counter += 1
        mv_iterator.clear_ext_trigger_events()
    # print("events: "+str(counter))
    '''

    print(mv_iterator) 
    mv_iterator.load_n_events(1)
    print(mv_iterator)
    '''

        # while True:
    # for evs in mv_iterator:
        # print(mv_iterator.get_ext_trigger_events())
        # print(evs)
        # Check if the event is an EXT_TRIGGER (1010) event
        # Extract the timestamp from the event
    '''
        timestamp = evs.timestamp()

        # Generate a frame using the timestamp
        on_demand_gen.generate(timestamp)

        # Get the generated frame and save it as a JPG file
        frame = on_demand_gen.get_latest_frame()
        frame.save_to_jpg(output_file+"frame{}".format(frame.timestamp()))
        '''



if __name__ == "__main__":
    main()
