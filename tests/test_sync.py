from collections import defaultdict
import time
import depthai as dai
import numpy as np
import cv2
from datetime import timedelta

pipeline = dai.Pipeline()

monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
color = pipeline.create(dai.node.ColorCamera)
stereo = pipeline.create(dai.node.StereoDepth)
sync = pipeline.create(dai.node.Sync)
stereo_sync = pipeline.create(dai.node.Sync)

xoutGrp = pipeline.create(dai.node.XLinkOut)
xoutGrp.setStreamName("xout")

stereo_out = pipeline.create(dai.node.XLinkOut)
stereo_out.setStreamName("stereo_sync")

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoLeft.setFps(30)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")
monoRight.setFps(30)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(False)
stereo.setSubpixel(False)
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = True
config.postProcessing.speckleFilter.speckleRange = 50
config.postProcessing.thresholdFilter.minRange = 200 # 0.2m
config.postProcessing.thresholdFilter.maxRange = 3_000  # 3m
stereo.initialConfig.set(config)
stereo.setPostProcessingHardwareResources(3, 3)

color.setCamera("color")
color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
color.setFps(30)

sync.setSyncThreshold(timedelta(milliseconds=int(1000/30/2)))
stereo_sync.setSyncThreshold(timedelta(milliseconds=int(1000/30/2)))

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

stereo.disparity.link(sync.inputs["depth"])
color.video.link(sync.inputs["rgb"])

sync.out.link(xoutGrp.input)


monoLeft.out.link(stereo_sync.inputs["left"])
monoRight.out.link(stereo_sync.inputs["right"])
stereo_sync.out.link(stereo_out.input)

# leftOut = pipeline.create(dai.node.XLinkOut)
# leftOut.setStreamName("left")
# rightOut = pipeline.create(dai.node.XLinkOut)
# rightOut.setStreamName("right")
# monoLeft.out.link(leftOut.input)
# monoRight.out.link(rightOut.input)



disparityMultiplier = 255.0 / stereo.initialConfig.getMaxDisparity()
with dai.Device(pipeline) as device:
    queue = device.getOutputQueue("xout", 10, False)
    stereo_queue = device.getOutputQueue("stereo_sync", 10, False)
    # q_left = device.getOutputQueue(name="left", maxSize=5, blocking=False)
    # q_right = device.getOutputQueue(name="right", maxSize=5, blocking=False)
    diffs = defaultdict(lambda: np.array([]))
    while True:
        start = time.time()

        # left = q_left.get()
        # cv2.imshow("left", left.getCvFrame())

        # right = q_right.get()
        # cv2.imshow("right", right.getCvFrame())

        stereo = stereo_queue.get()

        for name, msg in stereo:
            cv2.imshow(name, msg.getCvFrame())


        msgGrp = queue.get()
        
        for name, msg in msgGrp:
            frame = msg.getCvFrame()

            latencyMs = (dai.Clock.now() - msg.getTimestamp()).total_seconds() * 1000
            diffs[name] = np.append(diffs[name], latencyMs)
            print('[{}] Latency: {:.2f} ms, Average latency: {:.2f} ms, Std: {:.2f}'.format(name, latencyMs, np.average(diffs[name]), np.std(diffs[name])))

            if name == "disparity":
                print(frame.shape)
                frame = (frame * disparityMultiplier).astype(np.uint8)
                frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            cv2.imshow(name, frame)

       
        taken = time.time() - start
        print(f"FPS: {1/taken}")
        if cv2.waitKey(1) == ord("q"):
            break
