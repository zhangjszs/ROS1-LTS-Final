#!/usr/bin/env python3
"""Extract /resize_img_out frames from a rosbag and encode to MP4."""

import sys
import os
import numpy as np

# ROS environment
import rosbag
import av

OUTPUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "track_vision.mp4")
BAG_PATH = "/home/kerwin/rosbag/track.bag"
TOPIC = "/resize_img_out"


def bgr8_to_rgb(data, height, width):
    arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
    return arr[:, :, ::-1].copy()  # BGR -> RGB


def main():
    print(f"Opening bag: {BAG_PATH}")
    bag = rosbag.Bag(BAG_PATH, "r")

    total = bag.get_message_count(topic_filters=[TOPIC])
    print(f"Found {total} frames on {TOPIC}")

    container = av.open(OUTPUT_PATH, mode="w")

    stream = None
    frame_count = 0

    try:
        for topic, msg, t in bag.read_messages(topics=[TOPIC]):
            rgb = bgr8_to_rgb(bytes(msg.data), msg.height, msg.width)

            if stream is None:
                h, w = rgb.shape[:2]
                stream = container.add_stream("libx264", rate=20)
                stream.width = w
                stream.height = h
                stream.pix_fmt = "yuv420p"
                stream.options = {"crf": "23", "preset": "fast"}
                print(f"Video: {w}x{h} @20fps -> {OUTPUT_PATH}")

            av_frame = av.VideoFrame.from_ndarray(rgb, format="rgb24")
            av_frame.pts = frame_count
            for packet in stream.encode(av_frame):
                container.mux(packet)

            frame_count += 1
            if frame_count % 200 == 0:
                print(f"  {frame_count}/{total} frames processed...")

        # Flush encoder
        if stream is not None:
            for packet in stream.encode():
                container.mux(packet)

    finally:
        container.close()
        bag.close()

    print(f"\nDone! {frame_count} frames -> {OUTPUT_PATH}")
    size_mb = os.path.getsize(OUTPUT_PATH) / 1024 / 1024
    print(f"File size: {size_mb:.1f} MB")


if __name__ == "__main__":
    main()
