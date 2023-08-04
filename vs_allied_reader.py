import time
from threading import Thread, Event
from queue import Queue

import cv2
from vmbpy import *


def get_cam_ids():
    with VmbSystem.get_instance() as vmb:
        cams = vmb.get_all_cameras()

        assert len(cams) > 0, "no camera found" \
                              ""
        print('Cameras found: {}'.format(len(cams)))
        for ci, cam in enumerate(cams):
            print(f"camera {ci}: {cam.get_id()}\n")

        return [cam.get_id() for cam in cams]

class Allied_cam(Thread):

    def __init__(self, cam_id, exposure, gain, queue):
        super().__init__()
        self.killswitch = Event()
        self.queue = queue

        self.cam_id = cam_id
        self.camera = None
        self.frame = None
        self._fps = 0.0
        self.t0 = 0

        with VmbSystem.get_instance() as vmb:
            self.camera = vmb.get_camera_by_id(self.cam_id)
            print("camera id set")
            with self.camera as cam:
                self.setup_camera(cam, exposure, gain)
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

            self.queue.put_nowait(frame.as_opencv_image())

        cam.queue_frame(frame)

    def setup_camera(self, cam, exposure, gain):
        try:
            cam.ExposureTime.set(exposure)
            cam.Gain.set(gain)
        except (AttributeError, VmbFeatureError):
            raise AttributeError("failed to set exposure")

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
                    # self.camera.start_streaming(handler=self.frame_handler, buffer_count=5) # todo: indafare sul buffer_count
                    self.camera.start_streaming(handler=self.frame_handler)
                    self.killswitch.wait()
                finally:
                    self.camera.stop_streaming()

class AlliedReader():
    def __init__(self, cam_id, exposure, gain):
        super().__init__()
        self.queue = Queue()

        assert isinstance(cam_id, str), "cam_id has to be a string"
        self.cam_id = cam_id

        self.len = len(self.cam_id)
        self.cam = Allied_cam(cam_id=self.cam_id, exposure=exposure, gain=gain, queue=self.queue)
        ## possibile future synch script ##

        self.cam.start()


    def release(self):
        self.cam.stop()

    def get_next_frames(self):
        return self.queue.get()

    def __del__(self):
        self.release()



if __name__ == '__main__':

    cam_ids = get_cam_ids()
    idc = int(input(f"INPUT - Select camera from the above list: "))
    assert idc in range(0, len(cam_ids)), "wrong input"

    exposure = 175
    gain = 15

    cap = AlliedReader(cam_id=cam_ids[idc], exposure=exposure, gain=gain)

    while True:
        frame = cap.get_next_frames()

        cv2.imshow("cam", cv2.resize(frame, (0, 0), fx=0.2, fy=0.2))
        k = cv2.waitKey(1)

        if k == ord('q') or k == 27:
            break

        try:
            fps = cap.cam._fps
        except:
            fps = 0
        print(f"\rfps: {fps}", end='')

    cap.release()
