import subprocess
import numpy as np
import cv2 as cv
import detect_mod
import time

line_color = [255,0,0]
eucl_dist_line = 190
# low_hsv = [40,10,95] # green
# high_hsv = [110,75,245] #green

low_hsv = [40,10,95]
high_hsv = [110,75,245]

max_height = 1280
max_width = 720

# max_height = 640
# max_width = 480

color_mode = 0 # hsv
cut_top_pixels=0
detect = detect_mod.Detector(cut_top_pixels=cut_top_pixels, eps_line=.01, eps_symbol=.001, line_width=400, low_hsv=low_hsv, high_hsv=high_hsv, line_color=line_color, symbol_color=[0,0,255], eucl_dist_line=eucl_dist_line,
                 eucl_dist_symbol=150, resize_width=150, resize_height=150, mode="detect", debug=True)
video_device = "/dev/video2"
# gst_str = ('nvarguscamerasrc ! ' + 'video/x-raw(memory:NVMM), ' +
#           'width=(int)1280, height=(int)720, ' +
#           'format=(string)NV12, framerate=(fraction)30/1 ! ' + 
#           'nvvidconv flip-method=2 ! ' + 
#           'video/x-raw, width=(int){}, height=(int){}, ' + 
#           'format=(string)BGRx ! ' +
#           'videoconvert ! appsink').format(max_width, max_height)
# video_device = gst_str
# "mfw_v4lsrc ! ffmpegcolorspace ! video/x-raw-rgb ! appsink"
cap = cv.VideoCapture(video_device, cv.CAP_V4L2)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
clip_hist_percent = 1
if not cap.isOpened():
    print("Cannot open camera")
    exit()
   

def change_color_mode(value):
    global detect
    if value==1:
        detect.set_color_mode("hsv")
    elif value==0:
        detect.set_color_mode("euclidean")
    elif value==2:
        detect.set_color_mode("combined")

def change_contour_threshold(value):
    global detect
    detect.set_contour_threshold(value)

def change_cut_top_pixels(value):
    global detect
    detect.set_cut_top_pixels(value)


def change_gamma(value):
    global detect
    detect.set_gamma((value+1)/100)

def change_contrast(value):
    global detect
    detect.set_contrast(value)

def change_line_color_r(value):
    global line_color,detect
    line_color[2]=value
    detect.set_line_color(line_color)

def change_line_color_g(value):
    global line_color,detect
    line_color[1]=value
    detect.set_line_color(line_color)

def change_line_color_b(value):
    global line_color,detect
    line_color[0]=value
    detect.set_line_color(line_color)

def change_eucl_dist(value):
    global detect
    detect.set_eucl_dist_line(value)

def change_line_width(value):
    global detect
    detect.set_line_width(value)

def change_percent(value):
    global detect, clip_hist_percent
    detect.set_clip_hist_percent(value)
    clip_hist_percent = value

def change_hsv(value):
    global low_hsv, high_hsv
    low_hsv = [cv.getTrackbarPos('low_h','Title_window'), cv.getTrackbarPos('low_s','Title_window'), cv.getTrackbarPos('low_v','Title_window')]
    high_hsv = [cv.getTrackbarPos('high_h','Title_window'), cv.getTrackbarPos('high_s','Title_window'), cv.getTrackbarPos('high_v','Title_window')]
    detect.set_hsv_values(high_hsv,low_hsv)

def set_eposure(value):
    global cap
    # subprocess.check_call("v4l2-ctl -d " + video_device + " --set-ctrl=exposure_absolute="+str(value),shell=True)
    cap.set(cv.CAP_PROP_EXPOSURE, value   ) # exposure min=1 max=10000, step:1

def set_gain(value):
    global cap
    # subprocess.check_call("v4l2-ctl -d " + video_device + " --set-ctrl=gain="+str(value+1),shell=True)
    cap.set(cv.CAP_PROP_GAIN, value   ) # gain min=0 max=255 step=1

# Automatic brightness and contrast optimization with optional histogram clipping
def automatic_brightness_and_contrast(image, clip_hist_percent=25):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

    # Calculate grayscale histogram
    hist = cv.calcHist([gray],[0],None,[256],[0,256])
    hist_size = len(hist)

    # Calculate cumulative distribution from the histogram
    accumulator = []
    accumulator.append(float(hist[0]))
    for index in range(1, hist_size):
        accumulator.append(accumulator[index -1] + float(hist[index]))

    # Locate points to clip
    maximum = accumulator[-1]
    clip_hist_percent *= (maximum/100.0)
    clip_hist_percent /= 2.0

    # Locate left cut
    minimum_gray = 0
    while accumulator[minimum_gray] < clip_hist_percent:
        minimum_gray += 1

    # Locate right cut
    maximum_gray = hist_size -1
    while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
        maximum_gray -= 1

    # Calculate alpha and beta values
    alpha = 255 / (maximum_gray - minimum_gray)
    beta = -minimum_gray * alpha

    '''
    # Calculate new histogram with desired range and show histogram 
    new_hist = cv.calcHist([gray],[0],None,[256],[minimum_gray,maximum_gray])
    plt.plot(hist)
    plt.plot(new_hist)
    plt.xlim([0,256])
    plt.show()
    '''

    auto_result = cv.convertScaleAbs(image, alpha=alpha, beta=beta)
    return (auto_result, alpha, beta)

cv.namedWindow("Title_window")
cv.createTrackbar("r", "Title_window" , 0, 255, change_line_color_r)
cv.createTrackbar("g", "Title_window" , 255, 255, change_line_color_g)
cv.createTrackbar("b", "Title_window" , 0, 255, change_line_color_b)
cv.createTrackbar("dist", "Title_window" , eucl_dist_line, 1000, change_eucl_dist)
cv.createTrackbar("low_h", "Title_window" , low_hsv[0], 360, change_hsv)
cv.createTrackbar("low_s", "Title_window" , low_hsv[1], 255, change_hsv)
cv.createTrackbar("low_v", "Title_window" , low_hsv[2], 255, change_hsv)
cv.createTrackbar("high_h", "Title_window" , high_hsv[0], 360, change_hsv)
cv.createTrackbar("high_s", "Title_window" , high_hsv[1], 255, change_hsv)
cv.createTrackbar("high_v", "Title_window" , high_hsv[2], 255, change_hsv)
cv.createTrackbar("color_mode", "Title_window", color_mode, 2, change_color_mode)
cv.createTrackbar("cut_top_pixels", "Title_window", cut_top_pixels, 600, change_cut_top_pixels)

change_color_mode(color_mode)
# cv.createTrackbar("percent", "Title_window", clip_hist_percent, 100, change_percent)

# cv.createTrackbar("gamma", "Title_window", 160, 300, change_gamma)
# cv.createTrackbar("contrast", "Title_window", 127, 2*127, change_contrast)

# cv.createTrackbar("exposure", "Title_window", 1, 10000, set_eposure)
# cv.createTrackbar("gain", "Title_window", 0, 255, set_gain)
# subprocess.check_call("v4l2-ctl -d " + video_device + " --set-ctrl=exposure_auto=3",shell=True)

# cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 0)

cap.set(cv.CAP_PROP_FRAME_HEIGHT, max_height)
cap.set(cv.CAP_PROP_FRAME_WIDTH, max_width)
width  = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))  # float
height  = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))  # float

# codec = cv.VideoWriter.fourcc(*"MJPG") # MJPG
# cap.set(cv.CAP_PROP_FPS, 30.0)
# cap.set(cv.CAP_PROP_FOURCC, codec)
# cv.createTrackbar("line_width", "Title_window" , (width//2)**2, width*height, change_line_width)
detect.set_line_width((width//2)**2) #width//4

cv.createTrackbar("contour_threshold", "Title_window", 10000, 300000, change_contour_threshold)
detect.set_contour_threshold(10000)
# has_line=False
new_frame_t = 0
prev_frame_t = 0

while True:
    # Capture frame-by-frame
    new_frame_t = time.time()
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    
    # frame = frame[-int(frame.shape[0]//2):,:,]
    cv.imshow("Original",frame)
    # frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # frame,_,_ = automatic_brightness_and_contrast(frame, clip_hist_percent)
    # image_yuv = cv.cvtColor(frame, cv.COLOR_BGR2YUV)
    # image_yuv[:, :, 0] = cv.equalizeHist(image_yuv[:, :, 0])
    # frame = cv.cvtColor(image_yuv, cv.COLOR_YUV2BGR)
    # cv.imshow("eq", frame)

    
    has_line, output, angle, follow_symbol = detect.main(frame)
    if not has_line:
        print("We dont see any line")
        cv.imshow("output",output)
    else:
        cv.imshow("output",output[0][:,len(output[0][1])//2:,:])
        print("Line exists")
    # cv.imshow("Title_window",line_mask)
    # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # Display the resulting frame
    # if has_line:
    #     cv.imshow('output', output[0])
    # else:
    #     cv.imshow('input', frame)
    fps = 1/(new_frame_t-prev_frame_t) 
    prev_frame_t = new_frame_t
  
    
    print("fps", fps)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()