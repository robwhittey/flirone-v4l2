/*
 * Copyright (C) 2015-2016 Thomas <tomas123 @ EEVblog Electronics Community Forum>
 * Modified: Multi-camera support via serial number, thermal-only output, and auto-gain removed
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <libusb.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>

#include "jpeglib.h"
#include "plank.h"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <assert.h>

#define FRAME_WIDTH   160
#define FRAME_HEIGHT  120
#define FRAME_FORMAT  V4L2_PIX_FMT_RGB24

// -------------------------------------------------------
// AUTO-GAIN CONFIGURATION
// -------------------------------------------------------
static double TEMP_ABS_MIN   = 10.0;
static double TEMP_ABS_MAX   = 45.0;
static float  PERCENTILE_LOW  = 1.0f;
static float  PERCENTILE_HIGH = 99.0f;
static float  GAIN_SMOOTH     = 0.0f;
static int    AUTO_GAIN       = 1;
// -------------------------------------------------------

struct v4l2_capability vid_caps;
struct v4l2_format vid_format;

size_t framesize, linewidth;

int fdwr = 0;

#define VENDOR_ID  0x09cb
#define PRODUCT_ID 0x1996

static struct libusb_device_handle *devh = NULL;
int filecount = 0;
struct timeval t1, t2;

int FFC = 0;

#define BUF85SIZE 1048576
int buf85pointer = 0;
unsigned char buf85[BUF85SIZE];

static float smoothed_low  = -1.0f;
static float smoothed_high = -1.0f;

void print_format(struct v4l2_format *vid_format) {
    printf("     vid_format->type                =%d\n",  vid_format->type);
    printf("     vid_format->fmt.pix.width       =%d\n",  vid_format->fmt.pix.width);
    printf("     vid_format->fmt.pix.height      =%d\n",  vid_format->fmt.pix.height);
    printf("     vid_format->fmt.pix.pixelformat =%d\n",  vid_format->fmt.pix.pixelformat);
    printf("     vid_format->fmt.pix.sizeimage   =%u\n",  vid_format->fmt.pix.sizeimage);
    printf("     vid_format->fmt.pix.field       =%d\n",  vid_format->fmt.pix.field);
    printf("     vid_format->fmt.pix.bytesperline=%d\n",  vid_format->fmt.pix.bytesperline);
    printf("     vid_format->fmt.pix.colorspace  =%d\n",  vid_format->fmt.pix.colorspace);
}

#include "font5x7.h"
void font_write(unsigned char *fb, int x, int y, const char *string) {
    int rx, ry;
    while (*string) {
        for (ry = 0; ry < 5; ++ry)
            for (rx = 0; rx < 7; ++rx) {
                int v = (font5x7_basic[((*string) & 0x7F) - CHAR_OFFSET][ry] >> (rx)) & 1;
                fb[(y+rx) * 160 + (x + ry)] = v ? 0 : fb[(y+rx) * 160 + (x + ry)];
            }
        string++;
        x += 6;
    }
}

double temperature2raw(double tempC) {
    double RAWobj  = PlanckR1 / (PlanckR2 * (exp(PlanckB / (tempC + 273.15)) - PlanckF)) - PlanckO;
    double RAWrefl = PlanckR1 / (PlanckR2 * (exp(PlanckB / (TempReflected + 273.15)) - PlanckF)) - PlanckO;
    double RAW = (Emissivity * RAWobj + (1.0 - Emissivity) * RAWrefl);
    return RAW / 4.0;
}

double raw2temperature(unsigned short RAW) {
    RAW *= 4;
    double RAWrefl = PlanckR1 / (PlanckR2 * (exp(PlanckB / (TempReflected + 273.15)) - PlanckF)) - PlanckO;
    double RAWobj  = (RAW - (1 - Emissivity) * RAWrefl) / Emissivity;
    return PlanckB / log(PlanckR1 / (PlanckR2 * (RAWobj + PlanckO)) + PlanckF) - 273.15;
}

static int compare_ushort(const void *a, const void *b) {
    return (*(unsigned short *)a - *(unsigned short *)b);
}

unsigned short compute_percentile(unsigned short *arr, int n, float pct) {
    unsigned short *sorted = malloc(n * sizeof(unsigned short));
    memcpy(sorted, arr, n * sizeof(unsigned short));
    qsort(sorted, n, sizeof(unsigned short), compare_ushort);
    int idx = (int)((pct / 100.0f) * (n - 1));
    if (idx < 0) idx = 0;
    if (idx >= n) idx = n - 1;
    unsigned short val = sorted[idx];
    free(sorted);
    return val;
}


void compute_gain(unsigned short *pix, int n, int *out_low, int *out_high) {
    int raw_abs_low  = (int)temperature2raw(TEMP_ABS_MIN);
    int raw_abs_high = (int)temperature2raw(TEMP_ABS_MAX);

    if (raw_abs_low > raw_abs_high) {
        int t = raw_abs_low;
        raw_abs_low = raw_abs_high;
        raw_abs_high = t;
    }

    if (!AUTO_GAIN) {
        *out_low  = raw_abs_low;
        *out_high = raw_abs_high;
        return;
    }

    int pct_low  = (int)compute_percentile(pix, n, PERCENTILE_LOW);
    int pct_high = (int)compute_percentile(pix, n, PERCENTILE_HIGH);

    int combined_low  = (pct_low  > raw_abs_low)  ? pct_low  : raw_abs_low;
    int combined_high = (pct_high < raw_abs_high) ? pct_high : raw_abs_high;

    if (combined_high <= combined_low) {
        combined_high = combined_low + 1;
    }

    if (smoothed_low < 0.0f || smoothed_high < 0.0f) {
        smoothed_low  = (float)combined_low;
        smoothed_high = (float)combined_high;
    } else {
        smoothed_low  = GAIN_SMOOTH * smoothed_low  + (1.0f - GAIN_SMOOTH) * combined_low;
        smoothed_high = GAIN_SMOOTH * smoothed_high + (1.0f - GAIN_SMOOTH) * combined_high;
    }

    *out_low  = (int)smoothed_low;
    *out_high = (int)smoothed_high;

    if (*out_high <= *out_low) {
        *out_high = *out_low + 1;
    }
}


void startv4l2(const char *video_device) {
    int ret_code = 0;

    printf("using output device: %s\n", video_device);
    fdwr = open(video_device, O_RDWR);
    assert(fdwr >= 0);

    ret_code = ioctl(fdwr, VIDIOC_QUERYCAP, &vid_caps);
    assert(ret_code != -1);

    memset(&vid_format, 0, sizeof(vid_format));
    ret_code = ioctl(fdwr, VIDIOC_G_FMT, &vid_format);

    linewidth = FRAME_WIDTH * 3;
    framesize = FRAME_WIDTH * FRAME_HEIGHT * 3;

    vid_format.type                = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    vid_format.fmt.pix.width       = FRAME_WIDTH;
    vid_format.fmt.pix.height      = FRAME_HEIGHT;
    vid_format.fmt.pix.pixelformat = FRAME_FORMAT;
    vid_format.fmt.pix.sizeimage   = framesize;
    vid_format.fmt.pix.field       = V4L2_FIELD_NONE;
    vid_format.fmt.pix.bytesperline = linewidth;
    vid_format.fmt.pix.colorspace  = V4L2_COLORSPACE_SRGB;

    ret_code = ioctl(fdwr, VIDIOC_S_FMT, &vid_format);
    assert(ret_code != -1);
    print_format(&vid_format);
}

void closev4l2() {
    close(fdwr);
}

void vframe(char ep[], char EP_error[], int r, int actual_length,
            unsigned char buf[], unsigned char *colormap)
{
    time_t now1 = time(NULL);

    if (r < 0) {
        if (strcmp(EP_error, libusb_error_name(r)) != 0) {
            strcpy(EP_error, libusb_error_name(r));
            fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s\n",
                    ctime(&now1), ep, r, libusb_error_name(r));
            sleep(1);
        }
        return;
    }

    unsigned char magicbyte[4] = {0xEF, 0xBE, 0x00, 0x00};

    if ((strncmp(buf, magicbyte, 4) == 0) || ((buf85pointer + actual_length) >= BUF85SIZE))
        buf85pointer = 0;

    memmove(buf85 + buf85pointer, buf, actual_length);
    buf85pointer += actual_length;

    if (strncmp(buf85, magicbyte, 4) != 0) {
        buf85pointer = 0;
        printf("Reset buffer because of bad Magic Byte!\n");
        return;
    }

    uint32_t FrameSize   = buf85[ 8] + (buf85[ 9] << 8) + (buf85[10] << 16) + (buf85[11] << 24);
    uint32_t ThermalSize = buf85[12] + (buf85[13] << 8) + (buf85[14] << 16) + (buf85[15] << 24);
    uint32_t JpgSize     = buf85[16] + (buf85[17] << 8) + (buf85[18] << 16) + (buf85[19] << 24);
    uint32_t StatusSize  = buf85[20] + (buf85[21] << 8) + (buf85[22] << 16) + (buf85[23] << 24);

    if ((FrameSize + 28) > (uint32_t)buf85pointer)
        return;

    t1 = t2;
    gettimeofday(&t2, NULL);
    filecount++;
    buf85pointer = 0;

    unsigned short pix[160 * 120];
    int x, y, v;
    unsigned char *fb_proc  = malloc(160 * 120);
    unsigned char *fb_proc2 = malloc(160 * 120 * 3);
    memset(fb_proc, 128, 160 * 120);

    // Unpack RAW pixels
    for (y = 0; y < 120; ++y)
        for (x = 0; x < 160; ++x) {
            if (x < 80)
                v = buf85[2*(y * 164 + x) + 32] + 256 * buf85[2*(y * 164 + x) + 33];
            else
                v = buf85[2*(y * 164 + x) + 32 + 4] + 256 * buf85[2*(y * 164 + x) + 33 + 4];
            pix[y * 160 + x] = (unsigned short)v;
        }

    // Compute gain
    int gain_low, gain_high;
    compute_gain(pix, 160 * 120, &gain_low, &gain_high);

    int delta = gain_high - gain_low;
    if (delta < 150)
        delta = 150;

    // Scale pixels
    for (y = 0; y < 120; ++y)
        for (x = 0; x < 160; ++x) {
            int val = ((int)pix[y * 160 + x] - gain_low) * 255 / delta;
            if (val < 0)   val = 0;
            if (val > 255) val = 255;
            fb_proc[y * 160 + x] = (unsigned char)val;
        }

    // Colorise into RGB buffer
    for (y = 0; y < 120; ++y)
        for (x = 0; x < 160; ++x) {
            v = fb_proc[y * 160 + x];
            fb_proc2[3*y * 160 + x*3]     = colormap[3 * v];
            fb_proc2[3*y * 160 + x*3 + 1] = colormap[3 * v + 1];
            fb_proc2[3*y * 160 + x*3 + 2] = colormap[3 * v + 2];
        }

    // Write thermal RGB to loopback — skip FFC frames
    if (strncmp(&buf85[28 + ThermalSize + JpgSize + 17], "FFC", 3) == 0) {
        FFC = 1;
    } else {
        if (FFC == 1) {
            FFC = 0;
        } else {
            write(fdwr, fb_proc2, framesize);
        }
    }

    free(fb_proc);
    free(fb_proc2);
}

// -------------------------------------------------------
// Open a specific FLIR device by serial number
// -------------------------------------------------------
static struct libusb_device_handle *open_device_by_serial(libusb_context *ctx,
                                                           const char *target_serial)
{
    libusb_device **list;
    ssize_t count = libusb_get_device_list(ctx, &list);
    struct libusb_device_handle *handle = NULL;

    for (ssize_t i = 0; i < count; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) != 0) continue;
        if (desc.idVendor != VENDOR_ID || desc.idProduct != PRODUCT_ID) continue;

        if (libusb_open(list[i], &handle) != 0) {
            handle = NULL;
            continue;
        }

        unsigned char serial[64] = {0};
        int len = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                                                     serial, sizeof(serial));
        if (len > 0 && strcmp((char *)serial, target_serial) == 0) {
            printf("Matched device serial: %s\n", serial);
            libusb_free_device_list(list, 1);
            return handle;
        }

        libusb_close(handle);
        handle = NULL;
    }

    libusb_free_device_list(list, 1);
    fprintf(stderr, "Could not find FLIR device with serial: %s\n", target_serial);
    return NULL;
}

void print_bulk_result(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[]) {
    time_t now1 = time(NULL);
    int i;
    if (r < 0) {
        if (strcmp(EP_error, libusb_error_name(r)) != 0) {
            strcpy(EP_error, libusb_error_name(r));
            fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s\n",
                    ctime(&now1), ep, r, libusb_error_name(r));
            sleep(1);
        }
    } else {
        printf("\n: %s bulk read EP %s, actual length %d\nHEX:\n", ctime(&now1), ep, actual_length);
        for (i = 0; i < ((200 < actual_length) ? 200 : actual_length); i++)
            printf(" %02x", buf[i]);
        printf("\nSTRING:\n");
        for (i = 0; i < ((200 < actual_length) ? 200 : actual_length); i++)
            if (buf[i] > 31) printf("%c", buf[i]);
        printf("\n");
    }
}

int EPloop(const char *serial, const char *video_device, unsigned char *colormap) {
    int r = 1;
    libusb_context *ctx = NULL;

    r = libusb_init(&ctx);
    if (r < 0) { fprintf(stderr, "failed to initialise libusb\n"); exit(1); }

    devh = open_device_by_serial(ctx, serial);
    if (!devh) { fprintf(stderr, "Could not find/open device with serial %s\n", serial); goto out; }
    printf("Successfully opened FLIR device (serial: %s)\n", serial);

    r = libusb_set_configuration(devh, 3);
    if (r < 0) { fprintf(stderr, "libusb_set_configuration error %d\n", r); goto out; }
    printf("Successfully set usb configuration 3\n");

    r = libusb_claim_interface(devh, 0);
    if (r < 0) { fprintf(stderr, "libusb_claim_interface 0 error %d\n", r); goto out; }
    r = libusb_claim_interface(devh, 1);
    if (r < 0) { fprintf(stderr, "libusb_claim_interface 1 error %d\n", r); goto out; }
    r = libusb_claim_interface(devh, 2);
    if (r < 0) { fprintf(stderr, "libusb_claim_interface 2 error %d\n", r); goto out; }
    printf("Successfully claimed interfaces 0,1,2\n");

    unsigned char buf[1048576];
    int actual_length;
    char EP81_error[50] = "", EP83_error[50] = "", EP85_error[50] = "";
    unsigned char data[2] = {0, 0};

    startv4l2(video_device);

    int state = 1;

    while (1) {
        switch (state) {
            case 1:
                printf("stop interface 2 FRAME\n");
                r = libusb_control_transfer(devh, 1, 0x0b, 0, 2, data, 0, 100);
                if (r < 0) { fprintf(stderr, "Control Out error %d\n", r); return r; }

                printf("stop interface 1 FILEIO\n");
                r = libusb_control_transfer(devh, 1, 0x0b, 0, 1, data, 0, 100);
                if (r < 0) { fprintf(stderr, "Control Out error %d\n", r); return r; }

                printf("\nstart interface 1 FILEIO\n");
                r = libusb_control_transfer(devh, 1, 0x0b, 1, 1, data, 0, 100);
                if (r < 0) { fprintf(stderr, "Control Out error %d\n", r); return r; }

                state = 3;
                break;

            case 3:
                printf("\nAsk for video stream, start EP 0x85:\n");
                r = libusb_control_transfer(devh, 1, 0x0b, 1, 2, data, 2, 200);
                if (r < 0) { fprintf(stderr, "Control Out error %d\n", r); return r; }
                state = 4;
                break;

            case 4:
                r = libusb_bulk_transfer(devh, 0x85, buf, sizeof(buf), &actual_length, 100);
                if (actual_length > 0)
                    vframe("0x85", EP85_error, r, actual_length, buf, colormap);
                break;
        }

        r = libusb_bulk_transfer(devh, 0x81, buf, sizeof(buf), &actual_length, 10);

        r = libusb_bulk_transfer(devh, 0x83, buf, sizeof(buf), &actual_length, 10);
        if (strcmp(libusb_error_name(r), "LIBUSB_ERROR_NO_DEVICE") == 0) {
            fprintf(stderr, "EP 0x83 LIBUSB_ERROR_NO_DEVICE -> reset USB\n");
            goto out;
        }
    }

    libusb_release_interface(devh, 0);
    libusb_release_interface(devh, 1);
    libusb_release_interface(devh, 2);

out:
    libusb_reset_device(devh);
    libusb_close(devh);
    libusb_exit(ctx);
    return r >= 0 ? r : -r;
}

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr,
            "\nUsage:\n"
            "  flirone <palette.raw> <serial> <thermal_v4l2_device> "
            "[temp_min temp_max pct_low pct_high gain_smooth auto_gain]\n\n"
            "Example:\n"
            "  flirone palettes/Grayscale.raw T07O3Q00064 /dev/video2 10 45 1 99 0.0 1\n");
        exit(1);
    }

    const char *palette_path = argv[1];
    const char *serial       = argv[2];
    const char *video_device = argv[3];

    if (argc >= 5) TEMP_ABS_MIN    = atof(argv[4]);
    if (argc >= 6) TEMP_ABS_MAX    = atof(argv[5]);
    if (argc >= 7) PERCENTILE_LOW  = (float)atof(argv[6]);
    if (argc >= 8) PERCENTILE_HIGH = (float)atof(argv[7]);
    if (argc >= 9) GAIN_SMOOTH     = (float)atof(argv[8]);
    if (argc >= 10) AUTO_GAIN      = atoi(argv[9]);

    if (PERCENTILE_LOW < 0.0f)   PERCENTILE_LOW = 0.0f;
    if (PERCENTILE_LOW > 100.0f) PERCENTILE_LOW = 100.0f;
    if (PERCENTILE_HIGH < 0.0f)  PERCENTILE_HIGH = 0.0f;
    if (PERCENTILE_HIGH > 100.0f) PERCENTILE_HIGH = 100.0f;

    if (GAIN_SMOOTH < 0.0f) GAIN_SMOOTH = 0.0f;
    if (GAIN_SMOOTH > 1.0f) GAIN_SMOOTH = 1.0f;

    if (TEMP_ABS_MIN > TEMP_ABS_MAX) {
        double t = TEMP_ABS_MIN;
        TEMP_ABS_MIN = TEMP_ABS_MAX;
        TEMP_ABS_MAX = t;
    }

    unsigned char colormap[768];
    FILE *fp = fopen(palette_path, "rb");
    if (!fp) {
        fprintf(stderr, "Cannot open palette: %s\n", palette_path);
        exit(1);
    }
    fread(colormap, sizeof(unsigned char), 768, fp);
    fclose(fp);

    printf("Starting FLIR driver: serial=%s -> %s\n", serial, video_device);
    printf("Gain config: temp_min=%.2f temp_max=%.2f pct_low=%.2f pct_high=%.2f smooth=%.2f auto_gain=%d\n",
           TEMP_ABS_MIN, TEMP_ABS_MAX, PERCENTILE_LOW, PERCENTILE_HIGH, GAIN_SMOOTH, AUTO_GAIN);

    while (1)
        EPloop(serial, video_device, colormap);
}
