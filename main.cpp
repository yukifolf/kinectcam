// kinect_bridge.cpp
// Kinect v2 (libfreenect2) -> two v4l2loopback nodes:
//   IR  (512x424)  -> $KINECT_IR_DEV   (default /dev/video10)  : face-unlock device
//   RGB (1920x1080)-> $KINECT_RGB_DEV  (default /dev/video11)  : general webcam
//
// Both nodes are fed as RGB24. One process, one device handle, both streams.
//
// Env knobs:
//   KINECT_IR_DEV     v4l2loopback node for IR   (default /dev/video10)
//   KINECT_RGB_DEV    v4l2loopback node for RGB  (default /dev/video11)
//   KINECT_IR_AUTO    1 = auto-gain IR via 99th-percentile (default 0 = fixed)
//   KINECT_IR_GAIN    fixed divisor when AUTO=0 (default 256.0)
//   KINECT_COLOR_720  1 = downscale color to 1280x720 (default 0 = full 1080p)
//
// Build:
//   g++ -O2 -std=c++17 kinect_bridge.cpp -o kinect_bridge -lfreenect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>

#include <libusb-1.0/libusb.h>

#include <linux/videodev2.h>
#include <sys/inotify.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <csignal>
#include <vector>
#include <algorithm>
#include <atomic>

static std::atomic<bool> g_run{true};
static void on_sig(int) { g_run = false; }

static const char* env_or(const char* k, const char* d) {
    const char* v = getenv(k); return (v && *v) ? v : d;
}
static bool env_flag(const char* k) {
    const char* v = getenv(k); return v && (v[0] == '1' || v[0] == 't' || v[0] == 'T');
}

// Open a v4l2loopback node as a VIDEO_OUTPUT and fix its format to RGB24.
static int open_output(const char* dev, int w, int h) {
    int fd = open(dev, O_RDWR);
    if (fd < 0) { fprintf(stderr, "open %s: %s\n", dev, strerror(errno)); return -1; }
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width       = w;
    fmt.fmt.pix.height      = h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    fmt.fmt.pix.bytesperline = w * 3;
    fmt.fmt.pix.sizeimage    = w * h * 3;
    fmt.fmt.pix.colorspace   = V4L2_COLORSPACE_SRGB;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "VIDIOC_S_FMT %s: %s\n", dev, strerror(errno));
        close(fd); return -1;
    }
    return fd;
}

static bool write_all(int fd, const uint8_t* buf, size_t n) {
    while (n) {
        ssize_t w = write(fd, buf, n);
        if (w < 0) { if (errno == EINTR) continue; return false; }
        buf += w; n -= (size_t)w;
    }
    return true;
}

// IR float -> 8-bit gray, replicated to RGB24. Returns the gain it used.
static void ir_to_rgb24(const float* ir, int w, int h, std::vector<uint8_t>& out,
                        bool autogain, double fixed_div, double& ema_gain) {
    out.resize((size_t)w * h * 3);
    double gain;
    if (autogain) {
        // 99th-percentile via coarse histogram, robust against hot specular pixels.
        constexpr int BINS = 1024;
        uint32_t hist[BINS] = {0};
        const size_t N = (size_t)w * h;
        for (size_t i = 0; i < N; ++i) {
            int b = (int)(ir[i] * (BINS / 65536.0));
            if (b < 0) b = 0; else if (b >= BINS) b = BINS - 1;
            hist[b]++;
        }
        size_t target = (size_t)(N * 0.99), acc = 0; int p = BINS - 1;
        for (int b = 0; b < BINS; ++b) { acc += hist[b]; if (acc >= target) { p = b; break; } }
        double pval = (p + 1) * (65536.0 / BINS);
        if (pval < 1.0) pval = 1.0;
        double inst = 235.0 / pval;                 // map p99 -> ~235
        ema_gain = ema_gain > 0 ? (ema_gain * 0.9 + inst * 0.1) : inst;  // smooth -> no flicker
        gain = ema_gain;
    } else {
        gain = 1.0 / fixed_div;
    }
    const size_t N = (size_t)w * h;
    for (size_t i = 0; i < N; ++i) {
        int v = (int)(ir[i] * gain);
        if (v < 0) v = 0; else if (v > 255) v = 255;
        uint8_t g = (uint8_t)v;
        out[i*3+0] = g; out[i*3+1] = g; out[i*3+2] = g;
    }
}

// BGRX/RGBX (4B) -> RGB24, with optional nearest-neighbor downscale.
static void color_to_rgb24(const uint8_t* src, int sw, int sh, int src_fmt,
                           int dw, int dh, std::vector<uint8_t>& out) {
    out.resize((size_t)dw * dh * 3);
    // byte offsets within the 4-byte source pixel to land on R,G,B
    int ro, go, bo;
    if (src_fmt == libfreenect2::Frame::RGBX) { ro = 0; go = 1; bo = 2; }
    else                                      { ro = 2; go = 1; bo = 0; } // BGRX (default)
    for (int y = 0; y < dh; ++y) {
        int sy = (dh == sh) ? y : (int)((int64_t)y * sh / dh);
        const uint8_t* srow = src + (size_t)sy * sw * 4;
        uint8_t* drow = out.data() + (size_t)y * dw * 3;
        for (int x = 0; x < dw; ++x) {
            int sx = (dw == sw) ? x : (int)((int64_t)x * sw / dw);
            const uint8_t* p = srow + (size_t)sx * 4;
            drow[x*3+0] = p[ro];
            drow[x*3+1] = p[go];
            drow[x*3+2] = p[bo];
        }
    }
}

// The Xbox logo LED lives on the MCU's always-on rail; only USB autosuspend cuts its power.
// We find the device's sysfs path via libusb port numbers, then enable autosuspend at 0 ms.
static void kinect_led_off() {
    libusb_context* ctx = nullptr;
    if (libusb_init(&ctx) < 0) return;

    libusb_device** devs;
    ssize_t n = libusb_get_device_list(ctx, &devs);
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor d;
        if (libusb_get_device_descriptor(devs[i], &d) < 0) continue;
        if (d.idVendor != 0x045e || d.idProduct != 0x02c4) continue;

        int bus = libusb_get_bus_number(devs[i]);
        uint8_t ports[8];
        int np = libusb_get_port_numbers(devs[i], ports, sizeof(ports));
        if (np <= 0) break;

        // Build  /sys/bus/usb/devices/BUS-P1.P2...
        char base[256];
        int off = snprintf(base, sizeof(base), "/sys/bus/usb/devices/%d", bus);
        for (int p = 0; p < np; p++)
            off += snprintf(base + off, sizeof(base) - off, p ? ".%d" : "-%d", ports[p]);

        char path[320];
        snprintf(path, sizeof(path), "%s/power/control", base);
        FILE* f = fopen(path, "w"); if (f) { fputs("auto\n", f); fclose(f); }
        snprintf(path, sizeof(path), "%s/power/autosuspend_delay_ms", base);
        f = fopen(path, "w"); if (f) { fputs("0\n", f); fclose(f); }
        break;
    }
    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
}

int main() {
    signal(SIGINT,  on_sig);
    signal(SIGTERM, on_sig);

    // NVDEC VA-API driver lacks required YUV format support and segfaults on first frame.
    // Force libva to skip hardware drivers so libfreenect2 falls back to TurboJPEG.
    setenv("LIBVA_DRIVER_NAME", "__none__", 0);

    const char* ir_dev  = env_or("KINECT_IR_DEV",  "/dev/video10");
    const char* rgb_dev = env_or("KINECT_RGB_DEV", "/dev/video11");
    const bool  ir_auto = env_flag("KINECT_IR_AUTO");
    const double ir_div = atof(env_or("KINECT_IR_GAIN", "256.0"));
    const bool  c720    = env_flag("KINECT_COLOR_720");

    const int IRW = 512,  IRH = 424;
    const int CW  = 1920, CH = 1080;
    const int DW  = c720 ? 1280 : CW;
    const int DH  = c720 ? 720  : CH;

    libfreenect2::Freenect2 fn;
    if (fn.enumerateDevices() == 0) { fprintf(stderr, "no Kinect v2 found\n"); return 1; }

    libfreenect2::PacketPipeline* pipe = new libfreenect2::CpuPacketPipeline();
    libfreenect2::Freenect2Device* dev = fn.openDefaultDevice(pipe);
    if (!dev) { fprintf(stderr, "openDefaultDevice failed\n"); return 1; }

    libfreenect2::SyncMultiFrameListener listener(
        libfreenect2::Frame::Color | libfreenect2::Frame::Ir);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);   // IR shares the depth stream
    if (!dev->start()) { fprintf(stderr, "device start failed\n"); return 1; }
    fprintf(stderr, "Kinect %s up. IR->%s  RGB->%s (%dx%d)\n",
            dev->getSerialNumber().c_str(), ir_dev, rgb_dev, DW, DH);

    int ir_fd  = open_output(ir_dev,  IRW, IRH);
    int rgb_fd = open_output(rgb_dev, DW,  DH);
    if (ir_fd < 0 || rgb_fd < 0) { dev->stop(); dev->close(); return 1; }

    // Watch ir_dev for IN_CLOSE_NOWRITE: fires only when a read-only consumer (LinuxCamPAM)
    // closes the device, not when we close our own O_RDWR handle. That's our exit signal.
    int ino_fd = inotify_init1(IN_NONBLOCK);
    if (ino_fd >= 0)
        inotify_add_watch(ino_fd, ir_dev, IN_CLOSE_NOWRITE);

    std::vector<uint8_t> ir_buf, rgb_buf;
    double ema_gain = 0.0;
    libfreenect2::FrameMap frames;

    while (g_run) {
        if (!listener.waitForNewFrame(frames, 10 * 1000)) {  // 10s timeout
            fprintf(stderr, "frame timeout\n");
            continue;
        }
        libfreenect2::Frame* ir    = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame* color = frames[libfreenect2::Frame::Color];

        if (ir) {
            ir_to_rgb24((const float*)ir->data, (int)ir->width, (int)ir->height,
                        ir_buf, ir_auto, ir_div, ema_gain);
            write_all(ir_fd, ir_buf.data(), ir_buf.size());
        }
        if (color) {
            color_to_rgb24(color->data, (int)color->width, (int)color->height,
                           (int)color->format, DW, DH, rgb_buf);
            write_all(rgb_fd, rgb_buf.data(), rgb_buf.size());
        }
        listener.release(frames);

        // Non-blocking drain of inotify events; stop as soon as consumer closes the IR device.
        if (ino_fd >= 0) {
            alignas(struct inotify_event) char buf[sizeof(struct inotify_event) + NAME_MAX + 1];
            while (read(ino_fd, buf, sizeof(buf)) > 0)
                g_run = false;
        }
    }

    fprintf(stderr, "stopping...\n");
    if (!frames.empty())
        listener.release(frames);
    if (ino_fd >= 0) close(ino_fd);
    dev->stop();
    dev->close();
    kinect_led_off();
    close(ir_fd);
    close(rgb_fd);
    fprintf(stderr, "done.\n");
    return 0;
}