def parse_args():
    import argparse
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description='RAW to JPG.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--input-file', dest='input_path', required=True, help="Path to input RAW file")
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    counter = 0
    word = 0
    with open(args.input_path, 'rb') as f:
        while True:
            data = f.read(4) # read 64 bits (8 bytes) at a time
            if not data: # end of file
                break
            word = int.from_bytes(data, byteorder='big', signed=False) # convert bytes to integer
            # print(str(word)+"\n")
            if (word & 0xF0000000) == 0xA0000000: # check first 4 bits
                # print("Event Detected")
                counter += 1

    print(f"Total trigger events detected: {counter}")
    print("Last word encountered: {0:b}".format(word))

main()
