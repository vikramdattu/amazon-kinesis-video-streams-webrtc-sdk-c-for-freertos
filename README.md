# amazon-kinesis-video-streams-webrtc-sdk-c-for-freertos

This project demonstrates how to port [Amazon Kinesis Video WebRTC C SDK](https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c) to FreeRTOS.

Following Espressif boards are tested with the example as a reference platform:
- ESP32P4-Function-EV-Board v1.0B
- [ESP-S3-EYE](https://github.com/espressif/esp-who/blob/master/docs/en/get-started/ESP32-S3-EYE_Getting_Started_Guide.md)
- [ESP-Wrover-Kit](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-wrover-kit.html)

You may follow the same procedure to port to other hardware platforms.

## Clone the project

Please git clone this project using the command below.  This will git sub-module all depended submodules under main/lib.

```bash
git submodule update --init --recursive
```

## Reference platform

We use [ESP IDF 4.4.2](https://github.com/espressif/esp-idf/releases/tag/v4.4.2) SDK, the [ESP-Wrover-Kit](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-wrover-kit.html) and [ESP32-S3-EYE](https://github.com/espressif/esp-who/blob/master/docs/en/get-started/ESP32-S3-EYE_Getting_Started_Guide.md) as the reference platforms.
 - We recommend using `ESP32-S3-EYE` as it provides the camera feed and the example can be tested with the live feed.
 - For ESP32P4, please clone `release/v5.3` branch

Please git clone the ESP IDF 4.4.2 with following command

```
git clone -b v4.4.2 --recursive https://github.com/espressif/esp-idf.git esp-idf-v4.4.2
```
- This example is also tested for the latest ESP-IDF releases till `release/v5.3`. Please feel free to use your exising clone if you've already cloned IDF.

Make change to the following file:

```bash
components/esp_rom/include/esp32/rom/ets_sys.h
```

line 638 to 644 to following

```patch
+#ifndef STATUS
typedef enum {
    OK = 0,
    FAIL,
    PENDING,
    BUSY,
    CANCEL,
} STATUS;
+#endif
```

- The corresponding files for esp32s3 and esp32p4 are:

```bash
components/esp_rom/include/esp32s3/rom/ets_sys.h
components/esp_rom/include/esp32p4/rom/ets_sys.h
```

- ESP32-P4 is using hosted for Wi-Fi connectivity via esp32c6 interfaced board.
Please also do following change to the IDF, to fix a current known issue:
- On board ESP32-C6 needs to be built and flashed. Binary and instructions can be found [here](slave/README.md)

```patch
--- a/components/esp_wifi/src/wifi_netif.c
+++ b/components/esp_wifi/src/wifi_netif.c
@@ -69,7 +69,7 @@ static esp_err_t wifi_transmit(void *h, void *buffer, size_t len)
 static esp_err_t wifi_transmit_wrap(void *h, void *buffer, size_t len, void *netstack_buf)
 {
     wifi_netif_driver_t driver = h;
-#if CONFIG_SPIRAM
+#if 0 // CONFIG_SPIRAM
     return esp_wifi_internal_tx_by_ref(driver->wifi_if, buffer, len, netstack_buf);
 #else
     return esp_wifi_internal_tx(driver->wifi_if, buffer, len);
```

Please follow the [Espressif instructions](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html) to set up the IDF environment.

## Apply patches

Next, patch depended libraries for using with WebRTC.

### [wslay](https://github.com/tatsuhiro-t/wslay)

This project uses wslay as the websocket client. Please apply patches located in patch/wslay directory.

```
main/lib/wslay$ git am ../../../patch/wslay/*
```

### [libsrtp](https://github.com/cisco/libsrtp/releases/tag/v2.3.0)

This project uses v2.3.0 of libsrtp.  Please apply patches located in patch/libsrtp directory.

```
main/lib/libsrtp$ git am ../../../patch/libsrtp/*
```

### [usrsctp](https://github.com/sctplab/usrsctp/commit/939d48f9632d69bf170c7a84514b312b6b42257d)

Please apply patches as below.

```
main/lib/usrsctp$ git am ../../../patch/usrsctp/*
```

### [amazon-kinesis-video-streams-webrtc-sdk-c]

Please apply patches as below.

```
main/lib/amazon-kinesis-video-streams-webrtc-sdk-c$ git am ../../../patch/amazon-kinesis-video-streams-webrtc-sdk-c/*
```

If you run into problems when "git am" patches, you can use the following commands to resolve the problem. Or try "git am --abort" the process of git am, then "git apply" individual patches sequentially (in the order of the sequence number indicated by the file name).

```
$git apply --reject ../../../patch/problem-lib/problems.patch
// fix *.rej
$git add .
$git am --continue
```

## Configure the project

Select the target using the following command:

```bash
# for esp32
idf.py set-target esp32

# for esp32s3
idf.py set-target esp32s3

# for esp32p4
idf.py set-target esp32p4
```

Use menuconfig of ESP IDF to configure the project.

```bash
idf.py menuconfig
```

- These parameters under Example Configuration Options must be set.

- - ESP_WIFI_SSID
  - ESP_WIFI_PASSWORD
  - ESP_MAXIMUM_RETRY
  - AWS_ACCESS_KEY_ID
  - AWS_SECRET_ACCESS_KEY
  - AWS_DEFAULT_REGION
  - AWS_KVS_CHANNEL
  - AWS_KVS_LOG_LEVEL

- The modifications needed by this project can be seen in the auto-generated `sdkconfig` file located at the root directory.

### Video source

 - If you are using ESP32-S3-EYE OR ESP32P4_Function_EV_Board, please skip this section. ESP32-S3-EYE and ESP32P4_Function_EV_Board use live stream from the camera out of the box.

This project uses pre-recorded h.264 frame files for video streaming.  Please put the files on a SD card.  The files should look like:

/sdcard/h264SampleFrames/frame-%04d.h264.

 The “%04d” part of the file name should be replaced by a sequence number of the frame.

There are pre-generated video frame files under

    main/lib/amazon-kinesis-video-streams-webrtc-sdk-c/samples/h264SampleFrames/

ready to be copied to sdcard.

Please note that you can not use J-TAG and SD card simultaneously on ESP-Wrover-Kit because they share some pins.

[Generate video source](https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/blob/master/samples/h264SampleFrames/README.md)

Given a video file videotestsrc,  the following GStreamer command generates video frame files. If you want to reduce the number of video files, please modify related setting in sample code.



```
sh
gst-launch-1.0 videotestsrc pattern=ball num-buffers=1500 ! timeoverlay ! videoconvert ! video/x-raw,format=I420,width=1280,height=720,framerate=5/1 ! queue ! x264enc bframes=0 speed-preset=veryfast bitrate=128 byte-stream=TRUE tune=zerolatency ! video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline ! multifilesink location="frame-%04d.h264" index=1
```

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type `Ctrl-]`.)

See the Getting Started Guide of ESP IDF for full steps to configure and use ESP-IDF to build projects.

### Known limitations and issues

This project does not use audio at this point in time. When running on the ESP-Wrover-Kit, this project can only run at low frame rate and low bit rate.

The current implementation does not support data channel. Please check back later for availability of the data channel feature.

**[The m-line mismatch](https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/issues/803)**

When using the [WebRTC SDK Test Page](https://awslabs.github.io/amazon-kinesis-video-streams-webrtc-sdk-js/examples/index.html) to validate the demo, you may get m-line mismatch errors.  Different browsers have different behaviors.  To work around such errors, you need to run the [sample](https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-js/#Development) in [amazon-kinesis-video-streams-webrtc-sdk-js](https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-js), and disable audio functionality of audio. This patch disables the audio functionality.  A later release of this project may eliminate the need for this.

patch/amazon-kinesis-video-streams-webrtc-sdk-js/0001-diable-offerToReceiveAudio.patch`

Quickly steps to do above

1. Click https://awslabs.github.io/amazon-kinesis-video-streams-webrtc-sdk-js/examples/index.html

2. Right click on the page, click 'save as', make sure the format is 'Webpage, Complete" and save. The 'KVS WebRTC Test Page.html' file and 'KVS WebRTC Test Page_files' directory are save.

3. Enter the 'KVS WebRTC Test Page_files' directory and patch the 'viewer.js' file by commented out SDP offer to receive audio.

    console.log('[VIEWER] Creating SDP offer');
                await viewer.peerConnection.setLocalDescription(
                    await viewer.peerConnection.createOffer({
                        // offerToReceiveAudio: true,
                        offerToReceiveVideo: true,
                    }),
                );

4. Load the page

    KVS WebRTC Test Page.html

  for WebRTC testing

## Security

See [CONTRIBUTING](CONTRIBUTING.md#security-issue-notifications) for more information.

## License

This project is licensed under the Apache-2.0 License.
