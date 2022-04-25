import argparse
import time
import numpy as np
import cv2
import os
from pyueye import ueye
from yolo import YOLO
from PIL import  Image
## get frame from ids
#/etc/init.d/ueyeethdrc start

    #load parameter set from camera eeprom
    #ueye.is_ParameterSet(hCam, ueye.IS_PARAMETERSET_CMD_LOAD_EEPROM, None, 0)
def main():
    # set exposure
    #get exposure range
    fps = 0.0
    hCam = ueye.HIDS(0)#first available camera  1-254 The camera with its specified ID
    mem_ptr = ueye.c_mem_p()#pcImageMemory
    mem_id = ueye.int()#MEM_ID
    #Starts the driver and establishes the connection to the camera
    ret = ueye.is_InitCamera(hCam, None)
    if ret != ueye.IS_SUCCESS:
        print('init camera failed')
    else:
        print('init camera success')

    rangMin = ueye.double()
    rangMax = ueye.double()
    # increment
    rangInc = ueye.double()
    ueye.is_Exposure(hCam, ueye.IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, rangMin, ueye.sizeof(rangMin))
    ueye.is_Exposure(hCam, ueye.IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX, rangMax, ueye.sizeof(rangMax))
    ueye.is_Exposure(hCam, ueye.IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_INC, rangInc, ueye.sizeof(rangInc))
    print('rangMin:'+str(rangMin))
    print('rangMax:'+str(rangMax))
    print('rangInc:'+str(rangInc))
    #set exposure time in the range
    exposTime = ueye.double(rangMin+10) #any value you want in the range
    print('ExposTime:'+str(exposTime))
    ueye.is_Exposure(hCam, ueye.IS_EXPOSURE_CMD_SET_EXPOSURE, exposTime, ueye.sizeof(exposTime))

    #set diaplay mode
    ret = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB)
    #set color mode
    #ret = ueye.is_SetColorMode(hCam, ueye.IS_CM_BGR8_PACKED)
    nbpp = 24 #bits of per pixel. this value is associated with the color mode

    #get image size
    rect_aoi = ueye.IS_RECT()
    # Can be used to set the size and position of an "area of interest"(AOI) within an image
    ueye.is_AOI(hCam, ueye.IS_AOI_IMAGE_GET_AOI, rect_aoi, ueye.sizeof(rect_aoi))
    #allocate memory
    ret = ueye.is_AllocImageMem(hCam,
                            rect_aoi.s32Width.value,
                            rect_aoi.s32Height.value,
                            nbpp,
                            mem_ptr,
                            mem_id,
                            )
    #Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
    if ret != ueye.IS_SUCCESS:
        print('allocate image memory failed')
    else:
        print('allocate memory')

    #the allocated memory must be actived by set iamge
    ret = ueye.is_SetImageMem(hCam, mem_ptr, mem_id)
    if ret != ueye.IS_SUCCESS:
        print('set image memory failed')
    else:
        print('set image memory')
        flag = True
        count = 0

        while flag:
            #is_FreezeVideo excute once, capture one image
            ret = ueye.is_FreezeVideo(hCam, ueye.IS_WAIT)
            #ret = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT)
            if ret != ueye.IS_SUCCESS:
                print('capture failed')
            else:
                start_time = time.time()
                count += 1
                #print('capture %d images' %(count))
                #format memory data to OpenCV Mat
                #extract the data of our image memory
                # ueye.get_data(pcImageMemory, width, height, nBitsPerPixel, pitch, copy=False)
                array = ueye.get_data(mem_ptr,rect_aoi.s32Width.value,rect_aoi.s32Height.value,nbpp,rect_aoi.s32Width.value * int((nbpp + 7) / 8),True)
                frame = np.reshape(array, (rect_aoi.s32Height.value, rect_aoi.s32Width.value, 3))
                frame = cv2.resize(frame,(800,600), interpolation=cv2.INTER_CUBIC)
                # 格式转变，BGRtoRGB
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                # 转变成Image
                frame = Image.fromarray(np.uint8(frame))
                # 进行检测
                frame = np.array(yolo.detect_image(frame))
                # RGBtoBGR满足opencv显示格式
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                print('FPS: ', 1.0 / (time.time() - start_time), "fps")
                fps = (fps + (1. / (time.time() - start_time))) / 2
                #print("fps= %.2f" % (fps))
                frame = cv2.putText(frame, "fps= %.2f" % (fps), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.namedWindow('test Video')
                cv2.imshow("test Video", frame)
                #keycode = cv2.waitKey(1)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyWindow('test Video')
                    break
        #close camera
        if mem_ptr != None:
            ueye.is_FreeImageMem(hCam, mem_ptr, mem_id)
            mem_ptr = None
        ueye.is_ExitCamera(hCam)


if __name__ == '__main__':
    yolo = YOLO()
    main()