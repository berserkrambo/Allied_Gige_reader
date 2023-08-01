import time
from threading import Thread, Lock, Event

import cv2
import numpy as np
from vmbpy import *


def get_cam_ids():
    with VmbSystem.get_instance() as vmb:
        cams = vmb.get_all_cameras()

        assert len(cams) > 0, "no camera found" \
                              ""
        print('Cameras found: {}'.format(len(cams)))
        for cam in cams:
            print(f"acquiring from: {cam.get_id()}\n")

        return [cam.get_id() for cam in cams]

class Allied_cam(Thread):

    def __init__(self, cam_id):
        super().__init__()
        self.killswitch = Event()
        self.lock = Lock()

        self.cam_id = cam_id
        self.camera = None
        self.frame = None
        self._fps = 0.0
        self.t0 = 0

        with VmbSystem.get_instance() as vmb:
            self.camera = vmb.get_camera_by_id(self.cam_id)
            print("camera id set")
            with self.camera as cam:
                self.setup_camera(cam)
                print("camera setup")
                self.setup_pixel_format(cam)
                print("camera pixel set")

        print("end init")

    def frame_handler(self, cam: Camera, stream: Stream, frame: Frame):
        if frame.get_status() == FrameStatus.Complete:
            if self.t0 == 0:
                self.t0 = time.time()
            else:
                t = time.time()
                self._fps = 1 / (t - self.t0)
                self.t0 = t

            with self.lock:
                self.frame = frame.as_opencv_image().copy()

        cam.queue_frame(frame)

    def setup_camera(self, cam):
        try:
            cam.ExposureAuto.set('Continuous')

        except (AttributeError, VmbFeatureError):
            pass

        # Enable white balancing if camera supports it
        try:
            cam.BalanceWhiteAuto.set('Continuous')

        except (AttributeError, VmbFeatureError):
            pass

        # Try to adjust GeV packet size. This Feature is only available for GigE - Cameras.
        try:
            stream = cam.get_streams()[0]
            stream.GVSPAdjustPacketSize.run()
            while not stream.GVSPAdjustPacketSize.is_done():
                pass

        except (AttributeError, VmbFeatureError):
            pass

    def setup_pixel_format(self, cam):
        # Query available pixel formats. Prefer color formats over monochrome formats
        cam_formats = cam.get_pixel_formats()
        cam_color_formats = intersect_pixel_formats(cam_formats, COLOR_PIXEL_FORMATS)
        convertible_color_formats = tuple(f for f in cam_color_formats
                                          if PixelFormat.Bgr8 in f.get_convertible_formats())

        cam_mono_formats = intersect_pixel_formats(cam_formats, MONO_PIXEL_FORMATS)
        convertible_mono_formats = tuple(f for f in cam_mono_formats
                                         if PixelFormat.Bgr8 in f.get_convertible_formats())

        # if OpenCV compatible color format is supported directly, use that
        if PixelFormat.Bgr8 in cam_formats:
            cam.set_pixel_format(PixelFormat.Bgr8)

        # else if existing color format can be converted to OpenCV format do that
        elif convertible_color_formats:
            cam.set_pixel_format(convertible_color_formats[0])

        # fall back to a mono format that can be converted
        elif convertible_mono_formats:
            cam.set_pixel_format(convertible_mono_formats[0])

        else:
            raise BaseException('Camera does not support an OpenCV compatible format. Abort.')

    def stop(self):
        self.killswitch.set()

    def run(self):
        with VmbSystem.get_instance() as vmb:
            with self.camera:
                try:
                    self.camera.start_streaming(handler=self.frame_handler, buffer_count=1)
                    self.killswitch.wait()
                finally:
                    self.camera.stop_streaming()

    def get_next_frame(self):
        with self.lock:
            return self.frame

class AlliedReader():
    def __init__(self, cam_ids):
        super().__init__()

        if isinstance(cam_ids, str):
            cam_ids = [cam_ids]
        self.cam_ids = cam_ids

        self.len = len(self.cam_ids)
        self.cameras = [Allied_cam(cam_id=c_id) for c_id in self.cam_ids]
        self.stop = False
        self.frames = [None for c_id in self.cam_ids]
        ## possibile synch script ##

        [cb.start() for cb in self.cameras]


    def release(self):
        [cb.stop() for cb in self.cameras]

    def get_next_frames(self):
        self.frames = [cb.get_next_frame() for cb in self.cameras]
        return self.frames

    def __del__(self):
        self.release()




if __name__ == '__main__':

    cam_ids = get_cam_ids()
    caps = AlliedReader(cam_ids)

    current_cam = 1 # 1 to N perchè è più comodo sulla tastiera

    while True:
        t0 = time.time()

        frames = caps.get_next_frames()
        if any(frame is None for frame in frames):
            print("frame is None")
            time.sleep(1)
            continue

        frame = frames[current_cam-1]
        cv2.imshow("cam", cv2.resize(frame, (0,0), fx=0.2, fy=0.2))
        k = cv2.waitKey(1)

        if k == ord('q') or k == 27:
            break

        if k in range(1, len(cam_ids)):
            current_cam = k

        try:
            fps = caps.cameras[current_cam - 1]._fps
        except:
            fps = 0
        print(f"\rfps: {fps}", end='')

    caps.release()