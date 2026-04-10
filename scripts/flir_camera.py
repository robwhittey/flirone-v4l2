import cv2
import subprocess
import time
import os
import signal
import sys
import fcntl
import tempfile
import atexit
from contextlib import suppress

FLIRONE_DIR   = '/home/robwhittey/flirone-v4l2'
PALETTE       = 'palettes/Grayscale.raw'
VDEV_START_NR = 6


def _force_release_loopback(start_nr: int = VDEV_START_NR, n_cameras: int = 2):
    subprocess.run('sudo pkill -SIGKILL -f flirone || true',
                   shell=True, check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)

    devices = ' '.join(f'/dev/video{start_nr + i}' for i in range(n_cameras))
    subprocess.run(f'sudo fuser -k {devices} || true',
                   shell=True, check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.3)

    subprocess.run('sudo modprobe -r v4l2loopback',
                   shell=True, check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def setup_loopback(n_cameras: int, start_nr: int = VDEV_START_NR):
    _force_release_loopback(start_nr=start_nr, n_cameras=n_cameras)

    nrs    = ','.join(str(start_nr + i) for i in range(n_cameras))
    labels = ','.join(f'FLIR_{i+1}' for i in range(n_cameras))

    subprocess.run(
        f'sudo modprobe v4l2loopback devices={n_cameras} '
        f'video_nr={nrs} card_label="{labels}" exclusive_caps=1',
        shell=True, check=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


def _reset_terminal():
    with suppress(Exception):
        sys.stdout.write('\n')
        sys.stdout.flush()
        subprocess.run('stty sane', shell=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def teardown_loopback(start_nr: int = VDEV_START_NR, n_cameras: int = 2):
    _force_release_loopback(start_nr=start_nr, n_cameras=n_cameras)


def find_first_flir_serial() -> str:
    import glob
    for path in glob.glob('/sys/bus/usb/devices/*/idVendor'):
        try:
            vendor = open(path).read().strip()
            if vendor != '09cb':
                continue
            base = path.rsplit('/', 1)[0]
            serial_path = base + '/serial'
            if os.path.exists(serial_path):
                return open(serial_path).read().strip()
        except Exception:
            continue
    raise RuntimeError("No FLIR camera found via sysfs")


def _wait_device_ready(path: str, timeout: float = 10.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if os.path.exists(path):
            fd = -1
            try:
                fd = os.open(path, os.O_RDWR)
                fcntl.ioctl(fd, 0x80685600, b'\x00' * 104)
                return True
            except Exception:
                pass
            finally:
                if fd >= 0:
                    with suppress(Exception):
                        os.close(fd)
        time.sleep(0.1)
    return False


atexit.register(_reset_terminal)


class FLIRCamera:

    def __init__(self, vdev_thermal: str,
                 serial: str = None,
                 flirone_dir: str = FLIRONE_DIR,
                 palette: str = PALETTE,
                 setup_loopback_device: bool = True):
        self.serial       = serial or find_first_flir_serial()
        self.vdev_thermal = vdev_thermal
        self.flirone_dir  = flirone_dir
        self.palette      = palette
        self.flir_proc    = None
        self.cam          = None
        self._log_path    = None

        # Gain defaults — override with configure_gain() before start()
        self.temp_min    = 10.0
        self.temp_max    = 45.0
        self.pct_low     = 1.0
        self.pct_high    = 99.0
        self.gain_smooth = 0.0
        self.auto_gain   = True

    def configure_gain(self, temp_min: float = None, temp_max: float = None,
                       pct_low: float = None, pct_high: float = None,
                       gain_smooth: float = None, auto_gain: bool = None):
        """Configure thermal gain parameters. Must be called before start()."""
        if self.flir_proc is not None:
            raise RuntimeError("Cannot reconfigure gain while camera is running. Call stop() first.")
        if temp_min    is not None: self.temp_min    = temp_min
        if temp_max    is not None: self.temp_max    = temp_max
        if pct_low     is not None: self.pct_low     = pct_low
        if pct_high    is not None: self.pct_high    = pct_high
        if gain_smooth is not None: self.gain_smooth = gain_smooth
        if auto_gain   is not None: self.auto_gain   = auto_gain
        return self  # enables chaining: FLIRCamera(...).configure_gain(...).start()

    def start(self):
        vdev_nr = int(self.vdev_thermal.replace('/dev/video', ''))
        setup_loopback(n_cameras=1, start_nr=vdev_nr)

        self._start_backend()

        deadline = time.time() + 10.0
        while time.time() < deadline:
            self.cam = cv2.VideoCapture(self.vdev_thermal, cv2.CAP_V4L2)
            if self.cam.isOpened():
                return self
            self.cam.release()
            time.sleep(0.3)

        self.stop()
        raise RuntimeError(f"[{self.serial}] Failed to open {self.vdev_thermal} after retries")

    def _start_backend(self):
        if not _wait_device_ready(self.vdev_thermal):
            raise RuntimeError(f"[{self.serial}] {self.vdev_thermal} never became ready")

        log = tempfile.NamedTemporaryFile(
            mode='wb', prefix=f'flirone_{self.serial}_', suffix='.log', delete=False
        )
        self._log_path = log.name

        cmd = [
            './flirone',
            self.palette,
            self.serial,
            self.vdev_thermal,
            str(self.temp_min),
            str(self.temp_max),
            str(self.pct_low),
            str(self.pct_high),
            str(self.gain_smooth),
            str(int(self.auto_gain)),
        ]

        self.flir_proc = subprocess.Popen(
            cmd,
            cwd=self.flirone_dir,
            stdout=log,
            stderr=log,
            preexec_fn=os.setsid,
            stdin=subprocess.DEVNULL
        )
        log.close()

        t0 = time.time()
        while time.time() - t0 < 3.0:
            if self.flir_proc.poll() is not None:
                out = ''
                with suppress(Exception):
                    with open(self._log_path, 'r', errors='replace') as f:
                        out = f.read().replace('\r', '')
                raise RuntimeError(
                    f"[{self.serial}] flirone exited unexpectedly.\n{out}"
                )
            time.sleep(0.1)

    def read(self):
        if self.cam is None:
            return False, None
        return self.cam.read()

    def stop(self):
        if self.cam is not None:
            self.cam.release()
            self.cam = None
        try:
            self._stop_backend()
        finally:
            _reset_terminal()

    def _stop_backend(self):
        if self.flir_proc is None:
            return

        try:
            pgid = os.getpgid(self.flir_proc.pid)
        except Exception:
            pgid = None

        for sig in (signal.SIGTERM, signal.SIGKILL):
            if pgid:
                with suppress(Exception):
                    os.killpg(pgid, sig)
            else:
                with suppress(Exception):
                    self.flir_proc.send_signal(sig)
            try:
                self.flir_proc.wait(timeout=1.5)
                break
            except subprocess.TimeoutExpired:
                continue

        self.flir_proc = None

        if self._log_path:
            with suppress(Exception):
                os.unlink(self._log_path)
            self._log_path = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.stop()
        return False


class FLIRCameraArray:

    def __init__(self, camera_configs: list,
                 flirone_dir: str = FLIRONE_DIR,
                 palette: str = PALETTE,
                 setup_loopback_devices: bool = True):
        self.cameras = []
        self._loopback_owned = setup_loopback_devices
        self._start_nr = VDEV_START_NR
        self._n_cameras = len(camera_configs)

        if setup_loopback_devices:
            self._start_nr = min(
                int(cfg['vdev_thermal'].replace('/dev/video', ''))
                for cfg in camera_configs
            )
            setup_loopback(self._n_cameras, self._start_nr)

        for cfg in camera_configs:
            self.cameras.append(FLIRCamera(
                serial       = cfg['serial'],
                vdev_thermal = cfg['vdev_thermal'],
                flirone_dir  = cfg.get('flirone_dir', flirone_dir),
                palette      = cfg.get('palette', palette),
            ))

        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self._signal_handler)

    def configure_gain(self, **kwargs):
        """Apply the same gain config to all cameras. Must be called before start()."""
        for cam in self.cameras:
            cam.configure_gain(**kwargs)
        return self  # enables chaining: FLIRCameraArray(...).configure_gain(...).start()

    def _signal_handler(self, signum, frame):
        self._shutdown()
        sys.exit(0)

    def _shutdown(self):
        for cam in self.cameras:
            with suppress(Exception):
                cam.stop()
        if self._loopback_owned:
            teardown_loopback(self._start_nr, self._n_cameras)

    def start(self):
        try:
            for cam in self.cameras:
                cam.start()
        except RuntimeError as e:
            print(f"\n[ERROR] Camera init failed: {e}")
            print("Check camera is connected and /dev/videoX is correct.")
            self._shutdown()
            sys.exit(1)
        except Exception as e:
            print(f"\n[ERROR] Unexpected error during camera init: {e}")
            self._shutdown()
            sys.exit(1)
        print(f"FLIR cameras ready: {[c.vdev_thermal for c in self.cameras]}\n")
        return self

    def stop(self):
        try:
            self._shutdown()
        finally:
            _reset_terminal()

    def read_all_frames(self) -> dict:
        out = {}
        for cam in self.cameras:
            ret, frame = cam.read()
            if ret and frame is not None:
                out[cam.serial] = frame
        return out

    def read_all(self) -> dict:
        return {cam.serial: cam.read() for cam in self.cameras}

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.stop()
        return False

    def __len__(self):
        return len(self.cameras)

    def __iter__(self):
        return iter(self.cameras)


# -------------------------------------------------------
CAMERA_CONFIGS = [
    #{"serial": "T07O3Q00064", "vdev_thermal": "/dev/video6"},
    {"serial": "T07O3Q0001D", "vdev_thermal": "/dev/video7"},
]


def main():
    try:
        with FLIRCameraArray(CAMERA_CONFIGS) as array:
            while True:
                frames = array.read_all_frames()
                for serial, frame in frames.items():
                    resized = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
                    cv2.imshow(f"FLIR [{serial}]", resized)

                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
