import math
from math import *

import cv2
import numpy as np

# import classifier
import run_cython
from scripts import distance


class Detector(object):

    def __init__(self, cut_top_pixels=0, eps_line=.01, eps_symbol=.001, line_width=55, contour_threshold=500 , color_mode="hsv", low_hsv=[40,0,95], high_hsv=[110,75,245], line_color=(0,255,0), symbol_color=(0,0,255), eucl_dist_line=150,
                 eucl_dist_symbol=150, resize_width=150, resize_height=150, mode="detect", debug=False):
        super(Detector, self).__init__()
        self.color_mode = color_mode
        self.line_color = line_color
        self.symbol_color = symbol_color
        self.line_high_hsv = high_hsv
        self.line_low_hsv = low_hsv
        self.cut_top_pixels = cut_top_pixels  # 120
        self.contour_threshold = contour_threshold
        self.eps_line = eps_line
        self.eps_symbol = eps_symbol
        self.line_width = line_width  # average line_width
        self.intersection_turn = None  # save the point_to_go in case we lose sight of symbol
        self.eucl_dist_line = eucl_dist_line
        self.eucl_dist_symbol = eucl_dist_symbol
        self.resize_width = resize_width
        self.resize_height = resize_height
        self.mode = mode
        self.debug = debug
        self.gatherer_class = {}
        self.clip_hist_percent = 25
        self.brightness = 255
        self.contrast = 127
        self.gamma = 1.2

    def set_gamma(self, value):
        self.gamma = value
    def set_brightness(self,value):
        self.brightness = value

    def set_contrast(self,value):
        self.contrast = value

    def set_clip_hist_percent(self, value):
        self.clip_hist_percent = value
    
    def set_cut_top_pixels(self, value):
        self.cut_top_pixels = value
    
    def set_contour_threshold(self, value):
        self.contour_threshold = value

    def set_color_mode(self, value):
        # if value=="hsv" or value=="euclidean" or value=="combined":
        self.color_mode = value

    def set_eucl_dist_symbol(self, value):
        self.eucl_dist_symbol = value

    def set_eucl_dist_line(self, value):
        self.eucl_dist_line = value

    def set_line_color(self,line_color):
        self.line_color = line_color

    def set_symbol_color(self,symbol_color):
        self.symbol_color = symbol_color

    def set_hsv_values(self, high, low):
        self.line_high_hsv = high
        self.line_low_hsv = low

    def set_line_width(self,line_width):
        self.line_width = line_width

    def euclidean_detect(self, input_frame, eucl_dist): # (0,0,255) = RED
        height, width, pix = img.shape
        res = np.zeros(np.shape,dtype=np.uint8)
        b,g,r = color
        for j in range(height):
            for i in range(width):
                blue =  int(pow(img[j][i][0] - b,2))
                green = int(pow(img[j][i][1] - g,2))
                red = int(pow(img[j][i][2] - r,2))
                if sqrt(blue+green+red) < eucl_dist:
                    res[j][i] = img[j][i]
        return res
    
    def apply_brightness_contrast(self, input_img, brightness = 255, contrast = 127):
        brightness = self.map(self.brightness, 0, 510, -255, 255)
        contrast = self.map(self.contrast, 0, 254, -127, 127)

        if brightness != 0:
            if brightness > 0:
                shadow = brightness
                highlight = 255
            else:
                shadow = 0
                highlight = 255 + brightness
            alpha_b = (highlight - shadow)/255
            gamma_b = shadow

            buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
        else:
            buf = input_img.copy()

        if contrast != 0:
            f = float(131 * (contrast + 127)) / (127 * (131 - contrast))
            alpha_c = f
            gamma_c = 127*(1-f)

            buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)

        cv2.putText(buf,'B:{},C:{}'.format(brightness,contrast),(10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return buf

    def map(self, x, in_min, in_max, out_min, out_max):
        return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

    # Somehow I found the value of `gamma=1.6` to be the best in my case
    def adjust_gamma(self, image, gamma=1.6):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / self.gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
            for i in np.arange(0, 256)]).astype("uint8")

        # apply gamma correction using the lookup table
        return cv2.LUT(image, table)

    def auto_brightandcontrast(input_img, channel, clip_percent=1):
        histSize=256
        alpha=0
        beta=0
        minGray=0
        maxGray=0
        accumulator=[]

        if(clip_percent==0):
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(hist)
            return input_img
        else:
            hist = cv2.calcHist([input_img],[channel],None,[256],[0, 256])
            accumulator.insert(0,hist[0])    

            for i in range(1,histSize):
                accumulator.insert(i,accumulator[i-1]+hist[i])

            maxx=accumulator[histSize-1]
            minGray=0

            clip_percent=clip_percent*(maxx/100.0)
            clip_percent=clip_percent/2.0

            while(accumulator[minGray]<clip_percent[0]):
                minGray=minGray+1

            maxGray=histSize-1
            while(accumulator[maxGray]>=(maxx-clip_percent[0])):
                maxGray=maxGray-1

            inputRange=maxGray-minGray

            alpha=(histSize-1)/inputRange
            beta=-minGray*alpha

            out_img=input_img.copy()

            cv2.convertScaleAbs(input_img,out_img,alpha,beta)

            return out_img

    def fix_perspective(self, img, rect):
        # the order of the box points: bottom left, top left, top right,
        # bottom right
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # get width and height of the detected rectangle
        width = int(rect[1][0])
        height = int(rect[1][1])

        src_pts = box.astype("float32")
        # coordinate of the points in box points after the rectangle has been
        # straightened
        dst_pts = np.array([[0, height - 1],
                            [0, 0],
                            [width - 1, 0],
                            [width - 1, height - 1]], dtype="float32")

        # the perspective transformation matrix
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)

        # directly warp the rotated rectangle to get the straightened rectangle
        warped = cv2.warpPerspective(img, M, (width, height))
        return warped

    def auto_canny(self, image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)
        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)
        # return the edged image
        return edged

    def detect_symbol(self, input_frame):
        test = None
        # input_frame should be in BRG or else weird thinks can happen
        # symbol_img = np.array(
        #     run_cython.euclidean_detect(input_frame, self.symbol_color[0], self.symbol_color[1], self.symbol_color[2], self.eucl_dist_symbol))  # (0,0,255) = RED
        symbol_img = self.rgb_eucl(input_frame,self.symbol_color[0], self.symbol_color[1], self.symbol_color[2], self.eucl_dist_symbol)
        # symbol_img = euclidean_detect(input_frame, self.symbol_color, self.eucl_dist_symbol)  # (0,0,255) = RED slow_mode
        symbol_img = cv2.morphologyEx(symbol_img, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)))
        symbol_img = cv2.cvtColor(symbol_img, cv2.COLOR_BGR2GRAY)
        _, symbol_img = cv2.threshold(symbol_img, 0, 255, cv2.THRESH_BINARY)

        # differentiate between opencv version 3. and 4.
        if str(cv2.__version__).split(".")[0] == "3":
            _, contours, _ = cv2.findContours(symbol_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            contours, _ = cv2.findContours(symbol_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0 and cv2.contourArea(max(contours, key=cv2.contourArea)) > self.contour_threshold:
            c = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(c)
            symbol_cropped = symbol_img[y:y + h, x:x + w]

            (x, y), (w, h), angle = cv2.minAreaRect(c)
            rect = (x, y), (w, h), angle

            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .01 * peri, True)
            if len(approx) == 7:

                M = cv2.moments(symbol_img)
                top = (M["m10"] / M["m00"], M["m01"] / M["m00"])
                M = cv2.moments(self.auto_canny(symbol_img))
                bot = (M["m10"] / M["m00"], M["m01"] / M["m00"])
                symbol_img = cv2.cvtColor(symbol_img, cv2.COLOR_GRAY2BGR)
                # cv2.circle(symbol_img, (int(top[0]), int(top[1])), 5, [0, 0, 255], -1)
                # cv2.circle(symbol_img, (int(bot[0]), int(bot[1])), 5, [255, 0, 0], -1)
                box = np.int0(cv2.boxPoints(rect))
                p0 = (box[0][0], box[0][1])
                p1 = (box[1][0], box[1][1])
                p2 = (box[2][0], box[2][1])
                p3 = (box[3][0], box[3][1])
                if w > h:
                    top_point = distance.middle_point(p2, p3)
                    bot_point = distance.middle_point(p0, p1)
                else:
                    top_point = distance.middle_point(p2, p1)
                    bot_point = distance.middle_point(p0, p3)

                if distance.euclidean(top_point, top) < distance.euclidean(top_point, bot):
                    top = top_point
                    bot = bot_point
                else:
                    top = bot_point
                    bot = top_point
                # points_arrow = [a[0] for a in approx]
                # for a in points_arrow:
                #     cv2.circle(symbol_img, (int(a[0]), int(a[1])), 5, (0, 255, 0))
                return symbol_img, [bot, top], symbol_cropped
            else:
                symbol_img = cv2.cvtColor(symbol_img, cv2.COLOR_GRAY2BGR)
                return symbol_img, [], symbol_cropped
            # cv2.drawContours(symbol_img, [np.int0(cv2.boxPoints(rect))], 0, (255, 0, 0))

            # cv2.circle(symbol_img, (int(bot[0]), int(bot[1])), 5, (255, 0, 0))
            # cv2.circle(symbol_img, (int(top[0]), int(top[1])), 5, (0, 255, 0))

            # middle = distance.middle_point(top, bot)
            # cv2.circle(symbol_img, (int(middle[0]), int(middle[1])), 5, (0, 0, 255))
        symbol_img = cv2.cvtColor(symbol_img, cv2.COLOR_GRAY2BGR)
        return symbol_img, [], False

    def angle_between_points(self, points, flag=True):
        angle = math.degrees(atan2(points[1][1] - points[0][1], points[1][0] - points[0][0]))
        if flag:
            angle += 180 if angle < 0 else 0
        return angle

    def angle_between_vectors(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2' """
        unit_vector_1 = v1 / np.linalg.norm(v1)
        unit_vector_2 = v2 / np.linalg.norm(v2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        return np.arccos(dot_product)

    def points_vector(self, points):
        # compute points[0]point[1] vector
        return points[1][0] - points[0][0], points[1][1] - points[0][1]

    def distance_points(self, point, point_):
        return math.sqrt(math.pow(point[1] - point_[1], 2) + math.pow(point[0] - point_[0], 2))

    def distance_point_line(self, point, m, c):
        pass

    def distance_point_line(self, point, line):
        p3 = np.asarray(point)
        p2 = np.asarray(line[0])
        p1 = np.asarray(line[1])
        return np.linalg.norm(np.cross(p2 - p1, p1 - p3)) / np.linalg.norm(p2 - p1)

    def centroid(self, data):
        x, y = zip(*data)
        l = len(x)
        return sum(x) / l, sum(y) / l

    def add_group_by_distance(self, groups, p, distance):
        for n, g in enumerate(groups):
            try:
                g = self.centroid(g)
            except TypeError:
                g = self.centroid([g])
            if math.hypot(g[0] - p[0], g[1] - p[1]) < distance:
                groups[n].append(p)
                return groups
        groups.append([p])
        return groups

    def point_in_line(self, m, c, point):
        return True if (m * point[0]) + c == point[1] else False

    def line_equation(self, points):
        x_coords, y_coords = zip(*points)
        A = np.vstack([x_coords, np.ones(len(x_coords))]).T
        m, c = np.linalg.lstsq(A, y_coords)[0]
        print("Line Solution is y = {m}x + {c}".format(m=m, c=c))
        return m, c

    def skeletonize(self, img, morph=cv2.MORPH_RECT):
        # Skeletonization
        skel = np.zeros(img.shape, np.uint8)
        # ret, img = cv2.threshold(img, 127, 255, 0)
        # Get a Cross Shaped Kernel
        element = cv2.getStructuringElement(morph, (3, 3))
        while True:
            # Open the image
            open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
            # Substract open from the original image
            temp = cv2.subtract(img, open)
            # Erode the original image and refine the skeleton
            eroded = cv2.erode(img, element)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded.copy()
            # Step 5: If there are no white pixels left ie.. the image has been completely eroded, quit the loop
            if cv2.countNonZero(img) == 0:
                break

        # img = cv2.equalizeHist(skel)
        img = skel
        # cv2.imshow("skeleton", skel)
        return img

    def auto_exposure_control(self, input_image):
        scale = 1
        delta = 0
        d = 0.06
        l = 1000

        t_cur = current_exposure_time
        g_cur = current_gain
        small_to_large_image_size_ratio = 0.5
        input_image = cv2.resize(large_img, # original image
                       (0,0), # set fx and fy, not the final size
                       fx=small_to_large_image_size_ratio, 
                       fy=small_to_large_image_size_ratio, 
                       interpolation=cv2.INTER_NEAREST)
        anchors = [1/1.9, 1/1.5, 1/1.2, 1.0, 1.2, 1.5, 1.9]

        N = math.log(l*(1-d)+1)
        M = [np.zeros(input_image.shape)*len(anchors)]

        for i, a in enumerate(anchors):

            Iout = input_image**(1/a)
            grad_x = cv2.Sobel(gray, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
            # Gradient-Y
            # grad_y = cv.Scharr(gray,ddepth,0,1)
            grad_y = cv2.Sobel(gray, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
            
            
            abs_grad_x = cv2.convertScaleAbs(grad_x)
            abs_grad_y = cv2.convertScaleAbs(grad_y)
            
            
            grad = cv.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
            height,width = grad.shape

            for h in range(height):
                for w in width:
                    if grad[h][w]>=d:
                        M[i] += (1/N)*math.log(l*(grad[h][w]-d)+1)
        Y_opt = np.argmax(M)

    def autoAdjustments_with_convertScaleAbs(self, img):
        alow = img.min()
        ahigh = img.max()
        amax = 200
        amin = 55

        # calculate alpha, beta
        alpha = ((amax - amin) / (ahigh - alow))
        beta = amin - alow * alpha
        # perform the operation g(x,y)= α * f(x,y)+ β
        new_img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)

        return [new_img, alpha, beta]

    # finds line contours, 'paints' the image and returns a list of tuples which indicate each fitline[startPoint,stopPoint]
    def detect_line(self, original_image,low_hsv=[], high_hsv=[]):
        if self.color_mode=="hsv":
            # auto_brightandcontrast(original_image,)
            # auto_brightandcontrast(input_img, channel, clip_percent=1):
            print("HSV")
            original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
            line_img = cv2.inRange(original_image, (self.line_low_hsv[0],self.line_low_hsv[1],self.line_low_hsv[2]), (self.line_high_hsv[0],self.line_high_hsv[1],self.line_high_hsv[2]))
            line_img = cv2.bitwise_and(original_image, original_image, mask = line_img)
            line_img = cv2.cvtColor(line_img, cv2.COLOR_HSV2BGR)
        elif self.color_mode == "euclidean":
            print("RGB",self.line_color,"distance",self.eucl_dist_line)
            line_img = np.array(
            run_cython.euclidean_detect(original_image, self.line_color[0], self.line_color[1], self.line_color[2], self.eucl_dist_line))  # BGR: (255, 0, 0) = BLUE
        elif self.color_mode == "combined":
            print("combined")
            hsv_image_o = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
            hsv_image = cv2.inRange(hsv_image_o, (self.line_low_hsv[0],self.line_low_hsv[1],self.line_low_hsv[2]), (self.line_high_hsv[0],self.line_high_hsv[1],self.line_high_hsv[2]))
            hsv_image = cv2.bitwise_and(hsv_image_o, hsv_image_o, mask = hsv_image)
            hsv_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

            line_img = np.array(run_cython.euclidean_detect(original_image, self.line_color[0], self.line_color[1], self.line_color[2], self.eucl_dist_line))  # BGR: (255, 0, 0) = BLUE
            line_img = cv2.bitwise_or(line_img,hsv_image)

        line_img = cv2.morphologyEx(line_img, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)))
        line_img = cv2.morphologyEx(line_img, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)))
        # line_img = euclidean_detect(original_image, self.line_color, self.eucl_dist_line)  # (255, 0, 0) = BLUE
        # cv2.imshow("BLue", line_img)
        line_img = cv2.cvtColor(line_img, cv2.COLOR_BGR2GRAY)
        _, line_img = cv2.threshold(line_img, 0, 255, cv2.THRESH_BINARY_INV)
        # _, line_img = cv2.threshold(line_img, 0, 255, cv2.THRESH_BINARY)

        if str(cv2.__version__).split(".")[0] == "3":  # differentiate between opencv version 3. and 4.
            _, contours, hierarchy = cv2.findContours(line_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        else:
            contours, hierarchy = cv2.findContours(line_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # after applying mask make it bgr in order to add colors for edges and/or contours
        line_img = cv2.cvtColor(line_img, cv2.COLOR_GRAY2BGR)
        # list_exits = {"South": [], "North": [], "West": [], "East": [], "rest of the points":[]}
        list_exits = [[], [], [], [], []]

        # Either check all contours which are larger than 'threshold' OR check only the largest contour (prefer the 2nd)
        if len(contours) == 0 or cv2.contourArea(max(contours, key=cv2.contourArea)) < self.contour_threshold:
            return line_img, list_exits
        print("contours size", cv2.contourArea(max(contours, key=cv2.contourArea)))
        if self.debug:
            for c in contours:
                if cv2.contourArea(c)>self.contour_threshold:        
                    print(cv2.contourArea(c),self.contour_threshold)
                    cv2.drawContours(line_img, [c], -1, self.line_color,thickness=cv2.FILLED)  

        c = max(contours, key=cv2.contourArea)
        # x, y, w, h = cv2.boundingRect(c)
        # line_img = cv2.cvtColor(line_img, cv2.COLOR_GRAY2BGR)
        epsilon = self.eps_line * cv2.arcLength(c, False)
        approx = cv2.approxPolyDP(c, epsilon, True)
        pix_off = line_img.shape[0]/5  # trial and error
        for point in approx:
            if point[0][1] > line_img.shape[0] - pix_off:  # point on south border
                list_exits[0] = self.add_group_by_distance(list_exits[0], point[0], self.line_width)
            elif point[0][1] < pix_off:  # point on north border
                list_exits[1] = self.add_group_by_distance(list_exits[1], point[0], self.line_width)
            elif point[0][0] < pix_off:  # point on west border
                list_exits[2] = self.add_group_by_distance(list_exits[2], point[0], self.line_width)
            elif point[0][0] > line_img.shape[1] - pix_off:  # point on east border
                list_exits[3] = self.add_group_by_distance(list_exits[3], point[0], self.line_width)
            else:
                list_exits[4].append(point[0])
                # cv2.circle(line_img, (point[0][0], point[0][1]), 5, [0, 0, 255], -1)

        # find centroid of points of each border
        for i, groups in enumerate(list_exits[:4]):
            for j, p in enumerate(groups):
                list_exits[i][j] = self.centroid(p)  # overwrite list of points with their centroid
                # cv2.circle(line_img, list_exits[i][j], 5, [255, 0, 0], -1)

        # find centroid of the rest of the points. Possibly intersection point
        if list_exits[4]:
            list_exits[4] = [self.centroid(list_exits[4])]
            # cv2.circle(line_img, list_exits[4][0], 5, [0, 255, 0], -1)

        return line_img, list_exits

    def get_higher(self, list_points):
        return min(list_points, key=lambda x: x[1])

    def get_lower(self, list_points):
        return max(list_points, key=lambda x: x[1])

    def histogram_equalization(self, img_in):# segregate color streams
        b,g,r = cv2.split(img_in)
        h_b, bin_b = np.histogram(b.flatten(), 256, [0, 256])
        h_g, bin_g = np.histogram(g.flatten(), 256, [0, 256])
        h_r, bin_r = np.histogram(r.flatten(), 256, [0, 256])# calculate cdf    
        cdf_b = np.cumsum(h_b)  
        cdf_g = np.cumsum(h_g)
        cdf_r = np.cumsum(h_r)
        
        # mask all pixels with value=0 and replace it with mean of the pixel values 
        cdf_m_b = np.ma.masked_equal(cdf_b,0)
        cdf_m_b = (cdf_m_b - cdf_m_b.min())*255/(cdf_m_b.max()-cdf_m_b.min())
        cdf_final_b = np.ma.filled(cdf_m_b,0).astype('uint8')
      
        cdf_m_g = np.ma.masked_equal(cdf_g,0)
        cdf_m_g = (cdf_m_g - cdf_m_g.min())*255/(cdf_m_g.max()-cdf_m_g.min())
        cdf_final_g = np.ma.filled(cdf_m_g,0).astype('uint8')
        cdf_m_r = np.ma.masked_equal(cdf_r,0)
        cdf_m_r = (cdf_m_r - cdf_m_r.min())*255/(cdf_m_r.max()-cdf_m_r.min())
        cdf_final_r = np.ma.filled(cdf_m_r,0).astype('uint8')# merge the images in the three channels
        img_b = cdf_final_b[b]
        img_g = cdf_final_g[g]
        img_r = cdf_final_r[r]
      
        img_out = cv2.merge((img_b, img_g, img_r))# validation
        equ_b = cv2.equalizeHist(b)
        equ_g = cv2.equalizeHist(g)
        equ_r = cv2.equalizeHist(r)
        equ = cv2.merge((equ_b, equ_g, equ_r))
        #print(equ)
        if self.debug:
            cv2.imshow('output_name', equ)
        return img_out


    # Automatic brightness and contrast optimization with optional histogram clipping
    def automatic_brightness_and_contrast(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Calculate grayscale histogram
        hist = cv2.calcHist([gray],[0],None,[256],[0,256])
        hist_size = len(hist)

        # Calculate cumulative distribution from the histogram
        accumulator = []
        accumulator.append(float(hist[0]))
        for index in range(1, hist_size):
            accumulator.append(accumulator[index -1] + float(hist[index]))

        # Locate points to clip
        maximum = accumulator[-1]
        self.clip_hist_percent *= (maximum/100.0)
        self.clip_hist_percent /= 2.0

        # Locate left cut
        minimum_gray = 0
        while accumulator[minimum_gray] < self.clip_hist_percent:
            minimum_gray += 1

        # Locate right cut
        maximum_gray = hist_size -1
        while accumulator[maximum_gray] >= (maximum - self.clip_hist_percent):
            maximum_gray -= 1

        # Calculate alpha and beta values
        alpha = 255 / (maximum_gray - minimum_gray)
        beta = -minimum_gray * alpha

        '''
        # Calculate new histogram with desired range and show histogram 
        new_hist = cv2.calcHist([gray],[0],None,[256],[minimum_gray,maximum_gray])
        plt.plot(hist)
        plt.plot(new_hist)
        plt.xlim([0,256])
        plt.show()
        '''

        auto_result = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        return (auto_result, alpha, beta)

    def white_balance(self,img):
        result = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
        avg_a = np.average(result[:, :, 1])
        avg_b = np.average(result[:, :, 2])
        result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
        result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
        result = cv2.cvtColor(result, cv2.COLOR_LAB2RGB)
        return result

    def rgb_eucl(self,img,r,g,b,eucl_dist):

        img[:,:,0] *= int(((pow(img[:,:,0] - b,2)) + (pow(img[:,:,1] - b,2)) + (pow(img[:,:,1] - b,2)))/(eucl_dist+1))
        img[:,:,1] *= int(((pow(img[:,:,0] - b,2)) + (pow(img[:,:,1] - b,2)) + (pow(img[:,:,1] - b,2)))/(eucl_dist+1))
        img[:,:,2] *= int(((pow(img[:,:,0] - b,2)) + (pow(img[:,:,1] - b,2)) + (pow(img[:,:,1] - b,2)))/(eucl_dist+1))
        return img

    def main(self, input_image):
        if self.debug:
            print("Analysing frame")
            # np.set_printoptions(suppress=True)
        # input_image = self.adjust_gamma(input_image)
        # frame = input_image[int(self.cut_top_pixels):,:,:]
        frame = input_image#[input_image.shape[0]//2:,:,:]
        frame = self.white_balance(frame)
        # cv2.imshow("white_balance",frame)
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(10,10))
        # frame[:,:,0] = clahe.apply(frame[:,:,0])
        # frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR)

        # gaussian_3 = cv2.GaussianBlur(frame, (0, 0), 2.0)
        # frame = cv2.addWeighted(frame, 1.5, gaussian_3, -0.5, 0, frame)
        frame = cv2.normalize(frame, np.zeros(frame.shape), 0, 255, cv2.NORM_MINMAX)
        # frame = frame[int(frame.shape[0]//2):,:,:]
        
        # cv2.imshow("norm",frame)
        # frame,_,_ = self.automatic_brightness_and_contrast(input_image)
        # frame = input_image
        line_mask, linePoints = self.detect_line(frame)
        # return False, line_mask, 0, 0
        num_exits = sum([len(k) for k in linePoints[:4] if k])
        if num_exits < 1:
            # we dont see any line
            return False, line_mask, 0, 0

        symbols_mask, symbolPoints, cropped = frame, [], False #self.detect_symbol(frame)

        # if num_exits < 3 and not isinstance(cropped, bool):  # if we have a symbol and there is no intersection
        #     moments = cv2.HuMoments(cv2.moments(cropped)).flatten()
        #     temp = classifier.predict(moments, filename="scripts/train_knn.pkl")
        #     lookup = {0: 'man', 1: 'stairs', 2: 'telephone', 3: 'woman'}
        #     print(temp)
        #     try:
        #         self.gatherer_class[lookup[temp[0]]] += 1
        #     except KeyError:
        #         self.gatherer_class[lookup[temp[0]]] = 1
        #     if self.debug:
        #         print("Classify")
        #         print(max(self.gatherer_class.items(), key=lambda x: x[1]))
        # else:
        #     self.gatherer_class = {}

        line = line_mask
        symbols = symbols_mask

        # cv2.imshow("Blue Tracking", line)
        # cv2.imshow("Red Tracking", symbols)
        # cv2.waitKey(0)

        hasLine, lineDistance, searchRange, angle, point_to_go, follow_symbol = (False, 0, 0, 0, None, False)
        # detected symbol + intersection
        if symbolPoints and num_exits > 2 and self.intersection_turn is None and linePoints[4]:
            candidates = []
            if self.debug:
                print("choose one of the exits")
                print(linePoints)

            for i, point in enumerate(linePoints[:4]):
                for p in point:
                    lineVector = self.points_vector((linePoints[4][0], p))
                    symbolVector = self.points_vector((symbolPoints[0], symbolPoints[1]))
                    angle = self.angle_between_vectors(lineVector, symbolVector)
                    angle = math.degrees(angle)
                    if math.isnan(angle):
                        candidates.append((p, 90))
                    else:
                        candidates.append((p, angle))
            point_to_go = min(candidates, key=lambda y: y[1])
            point_to_go = point_to_go[0]
            follow_symbol = True
            self.intersection_turn = point_to_go
        elif num_exits > 2 and self.intersection_turn is not None:
            if self.debug:
                print("choose the closest to the previous chosen point")
            point_to_go = min(
                [min(l, key=lambda y: distance.euclidean(y, self.intersection_turn)) for l in linePoints if l],
                key=lambda y: distance.euclidean(y, self.intersection_turn))
            self.intersection_turn = point_to_go
        elif self.intersection_turn is not None:
            if self.debug:
                print("choose the closest to the previous chosen point (for one last time)")
            point_to_go = min(
                [min(l, key=lambda y: distance.euclidean(y, self.intersection_turn)) for l in linePoints if l],
                key=lambda y: distance.euclidean(y, self.intersection_turn))
            self.intersection_turn = None
        elif linePoints[4]:
            point_to_go = linePoints[4][0]
            if self.debug:
                print("choose the intersection")
                print(linePoints[4])
        else:
            point_to_go = self.get_higher(
                [self.get_higher(exit_points) for exit_points in [x for x in linePoints[:4] if len(x) > 0]])
            if self.debug:
                print("choose the Highest")

        robot_view_point = (line.shape[1] / 2, line.shape[0])
        if self.debug:
            cv2.circle(line, (int(point_to_go[0]),int(point_to_go[1])), 10, (255, 0, 255), 4)
        # lineDistance = self.distance_points(robot_view_point, point_to_go)
        # if angle_between_points([middle_point, higher_point]) < 0:
        #     lineDistance *= -1
        # searchRange = distance_points(middle_point, robot_view_point)

        angle = self.angle_between_points([robot_view_point, point_to_go])

        if angle > 90:
            if self.debug:
                print("Right")
            angle = -180 + angle
        else:
            if self.debug:
                print("Left")
        # left side: 0->90 from left to the center
        # right side: -90->0 from center to the right

        if self.debug:
            cv2.circle(line, (int(robot_view_point[0]),int(robot_view_point[1])), 50, (255, 0, 0), 5)
            cv2.line(line, (int(robot_view_point[0]),int(robot_view_point[1])), (int(point_to_go[0]),int(point_to_go[1])), (255, 195, 195), 1)
        # print(angle, math.radians(angle))
        hasLine = True

        if self.debug:
            print("Analysis complete. Angle to turn =", angle)
        frame = np.concatenate((symbols, line), axis=1)

        return hasLine, [frame, cropped], math.radians(angle), follow_symbol

