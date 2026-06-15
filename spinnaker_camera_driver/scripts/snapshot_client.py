#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Copyright 2026 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
r"""Example client for the spinnaker_camera_driver ``~/snapshot`` service.

Calls ``/<camera>/snapshot`` with a requested exposure_time (microseconds) and
gain (dB), handles the empty-image failure contract (the server returns an image
with ``width == 0`` on ANY failure), prints the result, and saves the returned
image to disk.

Run it with the colcon overlay sourced, e.g.::

    pixi run -- bash -c 'source .pixi/install/setup.bash && \\
        python3 flir_camera_driver/spinnaker_camera_driver/scripts/snapshot_client.py \\
        --camera flir_camera --exposure 5000 --gain 2.0 --output /tmp/snapshot.png'

Equivalent one-liner using the ros2 CLI (no image is saved, just prints the response)::

    pixi run -- bash -c 'source .pixi/install/setup.bash && \\
        ros2 service call /flir_camera/snapshot flir_camera_msgs/srv/Snapshot \\
        "{exposure_time: 5000.0, gain: 2.0}"'
"""

import argparse
import sys

from flir_camera_msgs.srv import Snapshot
import rclpy
from rclpy.node import Node


def _debayer_halfres(arr, enc):
    """Cheap half-resolution debayer of an HxW uint8 Bayer plane -> (H/2)x(W/2)x3."""
    import numpy as np

    # 2x2 mosaic letters at (row,col) = (0,0),(0,1),(1,0),(1,1)
    pattern = {
        'bayer_rggb8': 'RGGB', 'bayer_bggr8': 'BGGR',
        'bayer_grbg8': 'GRBG', 'bayer_gbrg8': 'GBRG',
    }[enc]
    h, w = arr.shape[0] // 2 * 2, arr.shape[1] // 2 * 2
    a = arr[:h, :w]
    quad = {
        pattern[0]: a[0::2, 0::2], pattern[1] + '1': a[0::2, 1::2],
        pattern[2] + '2': a[1::2, 0::2], pattern[3]: a[1::2, 1::2],
    }
    r = quad['R'].astype(np.uint16)
    b = quad['B'].astype(np.uint16)
    g = (quad['G1'].astype(np.uint16) + quad['G2'].astype(np.uint16)) // 2
    return np.dstack([r, g, b]).astype(np.uint8)


def save_image(msg, path):
    """Save a sensor_msgs/Image to disk; returns a human-readable status string.

    Falls back OpenCV -> Pillow -> NetPBM so the client works on a bare env.
    """
    width, height, enc, step = msg.width, msg.height, msg.encoding, msg.step
    raw = bytes(msg.data)

    try:
        import numpy as np
    except ImportError:
        with open(path + '.raw', 'wb') as f:
            f.write(raw)
        return f'numpy not available; wrote raw bytes to {path}.raw'

    chans = {
        'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4,
        'bayer_rggb8': 1, 'bayer_grbg8': 1, 'bayer_gbrg8': 1, 'bayer_bggr8': 1,
    }.get(enc, None)
    if chans is None:
        with open(path + '.raw', 'wb') as f:
            f.write(raw)
        return f"unsupported encoding '{enc}', wrote raw bytes to {path}.raw"

    arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, step)
    arr = arr[:, : width * chans].reshape(height, width, chans)

    try:
        import cv2

        if enc.startswith('bayer_'):
            code = {
                'bayer_rggb8': cv2.COLOR_BayerBG2BGR, 'bayer_bggr8': cv2.COLOR_BayerRG2BGR,
                'bayer_grbg8': cv2.COLOR_BayerGB2BGR, 'bayer_gbrg8': cv2.COLOR_BayerGR2BGR,
            }[enc]
            out = cv2.cvtColor(arr[:, :, 0], code)
        elif enc == 'rgb8':
            out = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif enc == 'rgba8':
            out = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        else:
            out = arr
        cv2.imwrite(path, out)
        return f'wrote {path} via OpenCV'
    except ImportError:
        pass

    try:
        from PIL import Image as PILImage

        if enc.startswith('bayer_'):
            PILImage.fromarray(_debayer_halfres(arr[:, :, 0], enc)).save(path)
            return f'wrote {path} via Pillow (half-res debayer; install opencv for full res)'
        if enc == 'mono8':
            PILImage.fromarray(arr[:, :, 0]).save(path)
        elif enc == 'rgb8':
            PILImage.fromarray(arr[:, :, :3]).save(path)
        elif enc.startswith('bgr'):
            PILImage.fromarray(arr[:, :, 2::-1]).save(path)
        return f'wrote {path} via Pillow'
    except ImportError:
        pass

    base = path.rsplit('.', 1)[0]
    if chans == 1 and not enc.startswith('bayer_'):
        outp, header, body = base + '.pgm', f'P5\n{width} {height}\n255\n'.encode(), \
            arr.reshape(height, width).tobytes()
    elif enc.startswith('bayer_'):
        rgb = _debayer_halfres(arr[:, :, 0], enc)
        outp = base + '.ppm'
        header = f'P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n'.encode()
        body = np.ascontiguousarray(rgb).tobytes()
    else:
        rgb = arr[:, :, 2::-1] if enc.startswith('bgr') else arr[:, :, :3]
        outp = base + '.ppm'
        header = f'P6\n{width} {height}\n255\n'.encode()
        body = np.ascontiguousarray(rgb).tobytes()
    with open(outp, 'wb') as f:
        f.write(header)
        f.write(body)
    return f'wrote {outp} (NetPBM; install opencv-python or pillow for PNG)'


def main():
    parser = argparse.ArgumentParser(description='Call the camera ~/snapshot service.')
    parser.add_argument('--camera', default='flir_camera', help='camera node name')
    parser.add_argument('--exposure', type=float, default=5000.0,
                        help='exposure time in microseconds (<=0 to leave unchanged)')
    parser.add_argument('--gain', type=float, default=2.0,
                        help='gain in dB (pass nan to leave unchanged)')
    parser.add_argument('--leave-gain', action='store_true',
                        help='leave gain unchanged (sends NaN)')
    parser.add_argument('--output', default='/tmp/snapshot.png', help='output image path')
    parser.add_argument('--wait', type=float, default=5.0,
                        help='seconds to wait for the service to appear')
    args = parser.parse_args()

    rclpy.init()
    node = Node('snapshot_client')
    srv_name = f'/{args.camera}/snapshot'
    client = node.create_client(Snapshot, srv_name)

    node.get_logger().info(f'waiting for service {srv_name} ...')
    if not client.wait_for_service(timeout_sec=args.wait):
        node.get_logger().error(f'service {srv_name} not available')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    req = Snapshot.Request()
    req.exposure_time = float(args.exposure)
    req.gain = float('nan') if args.leave_gain else float(args.gain)
    node.get_logger().info(
        f'requesting snapshot exposure_time={req.exposure_time}us gain={req.gain}dB')

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.wait + 10.0)
    if not future.done():
        node.get_logger().error('service call did not return in time')
        node.destroy_node()
        rclpy.shutdown()
        return 3

    resp = future.result()
    img = resp.image

    # failure contract: result == ERROR (with an empty image) on any failure
    if resp.result != Snapshot.Response.SUCCESS:
        node.get_logger().error(
            'SNAPSHOT FAILED: server returned result=ERROR (empty image). '
            'Check the server log for the specific reason '
            '(out-of-range exposure/gain, verification timeout, trigger error, not streaming).')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    ci = resp.camera_info
    print('SNAPSHOT OK')
    print(f'  size       : {img.width} x {img.height}')
    print(f'  encoding   : {img.encoding}')
    print(f'  step       : {img.step} bytes/row  (data: {len(img.data)} bytes)')
    print(f'  stamp      : {img.header.stamp.sec}.{img.header.stamp.nanosec:09d}')
    print(f'  frame_id   : {img.header.frame_id}')
    print(f'  camera_info: {ci.width} x {ci.height}, model "{ci.distortion_model}"')

    status = save_image(img, args.output)
    print(f'  saved      : {status}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
