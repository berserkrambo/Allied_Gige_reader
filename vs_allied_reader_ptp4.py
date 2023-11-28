import copy
import queue
import threading
import os
import cv2
import numpy
import numpy as np
import time

from vmbpy import *


def setup_pixel_format(cam):
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


def setup_camera(cam, exposure, gain, fps, operationMode):
    cam.UserSetLoad.run()

    cam.TriggerMode.set("Off")

    cam.ExposureTime.set(exposure)
    cam.Gain.set(gain)
    setup_pixel_format(cam)

    cam.AcquisitionFrameRateEnable.set('True')
    cam.AcquisitionFrameRate.set(fps)

    stream = cam.get_streams()[0]
    stream.GVSPAdjustPacketSize.run()
    while not stream.GVSPAdjustPacketSize.is_done():
        pass

    # cam.PtpEnable.set("False")
    cam.PtpEnable.set("True")
    cam.PtpOperationMode.set(operationMode)

# Thread Objects
class Allied_cam(threading.Thread):
    def __init__(self, cam, exposure, gain, fps, master_id, queue):
        super().__init__()
        self.cam = cam
        self.exposure = exposure
        self.gain = gain
        self.fps = fps
        self.master_id = master_id
        self.queue = queue
        self.ptpSet = False
        self.ptpStatus = ""
        self.killswitch = threading.Event()

        # self.frame = None
        self._fps = 0.0
        self.t0 = 0

        with self.cam:
            self.cam_id = self.cam.get_id()
            operationMode = "Master" if master_id == self.cam_id else "Slave"
            setup_camera(self.cam, self.exposure, self.gain, self.fps, operationMode)
            print(f"{self.cam_id}, setup_end")

    def frame_handler(self, cam, stream, frame):
        # This method is executed within VmbC context. All incoming frames
        # are reused for later frame acquisition. If a frame shall be queued, the
        # frame must be copied and the copy must be sent, otherwise the acquired
        # frame will be overridden as soon as the frame is reused.
        # print("call")
        frame_status = frame.get_status()
        print(frame_status)
        if frame_status == FrameStatus.Complete:
            frame_cpy = copy.deepcopy(frame)
            # frame_cpy = frame_cpy.as_opencv_image()

            if self.t0 == 0:
                self.t0 = time.time()
            else:
                t = time.time()
                self._fps = 1 / (t - self.t0)
                self.t0 = t

            self.queue.put_nowait(frame_cpy)

        cam.queue_frame(frame)

    def stop(self):
        self.killswitch.set()

    def run(self):
        with self.cam:
            while self.ptpStatus == "":
                time.sleep(0.1)
                print(f"{self.cam_id}, syncing....{self.ptpStatus}")
                self.cam.PtpDataSetLatch.run()
                self.ptpStatus = self.cam.PtpStatus.get()
            print(f"{self.cam.get_id()} status {self.ptpStatus}")

            self.cam.start_streaming(self.frame_handler)
            print("streaming started")
            self.killswitch.wait()

            self.cam.stop_streaming()


class AlliedWrapper():
    def __init__(self, exposure, gain, fps):
        self.master_id = "DEV_000A47000A72"
        self.exposure = exposure
        self.gain = gain
        self.fps = fps

        self.vmb = VmbSystem.get_instance()

        with self.vmb:
            # Construct FrameProducer threads for all detected cameras
            self.all_cam = self.vmb.get_all_cameras()

            self.len_cams = len(self.all_cam)
            self.queues = [queue.Queue() for ci in range(self.len_cams)]
            self.frames = [None for ci in range(self.len_cams)]

            # setup cameras
            self.cams = []
            for ci, cam in enumerate(self.all_cam):
                cm = Allied_cam(cam, self.exposure, self.gain, self.fps, self.master_id, self.queues[ci])
                self.cams.append(cm)

            print("starting threads")
            [c.start() for c in self.cams]

            folder_num = 0
            while True:
                print('getting frames')
                frames = [q.get() for q in self.queues]
                frames = [frame.as_opencv_image() for frame in frames]
                print("frames dequeued")

                cv2.imshow("cam", cv2.resize(np.hstack(frames), (0, 0), fx=0.2, fy=0.2))
                """
                img_num = 0
                folder = "../demo_imgs/test_movimento/" + str(folder_num) + "/"
                os.makedirs(folder, exist_ok=True)
                folder_num += 1

                for f_index in range(len(frames)):
                    filename = folder + str(exposure) + "_" + str(
                        self.cams[img_num].cam_id + ".png")
                    cv2.imwrite(filename, frames[f_index])
                    img_num += 1
                """
                k = cv2.waitKey(1)

                if k == ord('q') or k == 27:
                    cv2.destroyAllWindows()
                    [cam.stop() for cam in self.cams]
                    [cam.join() for cam in self.cams]
                    break

    def get_next_frame(self):
        return self.frames

    def get_fps(self):
        return [cam._fps for cam in self.cams]

    def release(self):
        [cam.stop() for cam in self.cams]
        [cam.join() for cam in self.cams]

    def stop(self):
        self.release()

    def __del__(self):
        self.release()


def main():
    import time
    exposure = 5000
    gain = 0
    fps = 20

    allied_reader = AlliedWrapper(exposure, gain, fps)
    """
    while True:
        frames = allied_reader.get_next_frame()

        if any(f is None for f in frames):
            continue

        cv2.imshow("cam", cv2.resize(np.hstack(frames), (0, 0), fx=0.2, fy=0.2))
        k = cv2.waitKey(1)

        if k == ord('q') or k == 27:
            break

        fps = [str(fps) for fps in allied_reader.get_fps()]
        fps = " - ".join(fps)

        print(f"\rfps: {fps}", end='')
    """
    allied_reader.release()



if __name__ == '__main__':
    main()
