import time
from vmbpy import *
import threading
import copy
from path import Path
import cv2

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


def setup_camera(cam, exposure, gain):
    cam.ExposureTime.set(exposure)
    cam.Gain.set(gain)
    setup_pixel_format(cam)

    stream = cam.get_streams()[0]
    stream.GVSPAdjustPacketSize.run()
    while not stream.GVSPAdjustPacketSize.is_done():
        pass


class AlliedCamera(threading.Thread):
    def __init__(self, camera, exposure, gain):
        super().__init__()
        self.cam = camera
        self.killswitch = threading.Event()
        self.acquired = threading.Event()
        self.last_frame = None

        self.cam._open()

        self.id = self.cam.get_id()
        print(self.id)
        self.cam.UserSetLoad.run()
        setup_camera(self.cam, exposure, gain)
        self.cam.TriggerSource.set('Software')
        self.cam.TriggerSelector.set('FrameStart')
        self.cam.TriggerMode.set('On')
        self.cam.AcquisitionMode.set('Continuous')
        # self.cam.AcquisitionMode.set('SingleFrame')

    def __call__(self,cam, stream, frame):
        print("triggerred")
        self.last_frame = copy.deepcopy(frame)
        self.last_frame = self.last_frame.as_opencv_image()
        cam.queue_frame(frame)
        self.acquired.set()

        print('camera {}, Frame acquired: {}'.format(cam.get_id(), frame), flush=True)

    def run(self):

        self.cam.start_streaming(self)
        self.killswitch.wait()

        self.cam.stop_streaming()
        self.cam._close()

    def get_frame(self):
        self.cam.TriggerSoftware.run()
        self.acquired.wait()
        self.acquired.clear()

        return self.id, self.last_frame


    def release(self):
        self.killswitch.set()

    def stop(self):
        self.release()

    def __del__(self):
        self.release()

class AlliedWrapper():
    def __init__(self, exposure, gain):
        self.vmb = VmbSystem.get_instance()
        self.vmb.__enter__()

        camera_list = self.vmb.get_all_cameras()
        self.caps = [AlliedCamera(cam, exposure, gain) for cam in camera_list]
        print("cam setup end")

        [c.start() for c in self.caps]
        print("cam started, sleeping 5 sec to be ready...")
        time.sleep(5)

        print("Now ready...")

    def get_frames(self):
        return [c.get_frame() for c in self.caps]

    def release(self):
        [c.release() for c in self.caps]
        [c.join() for c in self.caps]
        self.vmb.__exit__(None, None, None)

    def stop(self):
        self.release()

    def __del__(self):
        self.release()


def get_input() -> str:
    prompt = 'Press \'a\' to send action command. Press \'q\' to stop example. Enter:'
    print(prompt, flush=True)
    return input()

def main ():

    save_path = Path(__file__).parent.parent
    exposure = 5000
    gain = 0

    allied_cams = AlliedWrapper(exposure=exposure, gain=gain)

    folder_num = 0
    while True:
        ch = get_input()

        if ch == 'q':
            allied_cams.release()
            break

        elif ch == 'a':
            frames = allied_cams.get_frames()

            img_num = 0
            folder = save_path / "demo_imgs_ricca" / "misurazione28-11-2023"  / str(folder_num)
            folder.makedirs_p()
            folder_num += 1

            for cam_id, frame in frames:
                filename = folder /  f"{exposure}_{cam_id}.png"
                cv2.imwrite(filename, frame)
                img_num += 1


if __name__ == '__main__':
    main()