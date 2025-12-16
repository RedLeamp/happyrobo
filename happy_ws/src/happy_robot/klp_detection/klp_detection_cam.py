#! /usr/bin/env python3
import torch
import easyocr
import numpy as np
import cv2
from PIL import ImageFont, ImageDraw, Image
import rospy
from sensor_msgs.msg import Image as ImageMSG
from cv_bridge import CvBridge
import os

class KLPDetection:
    def __init__(self):
        self.image = None
        self.br = CvBridge()
        # Publishers
        self.pub = rospy.Publisher('/rgb/detected_image', ImageMSG, queue_size=10)
        # Subscribers
        self.sub = rospy.Subscriber("/rgb/image_raw", ImageMSG, self.callback)
        # Torch
        self.fontpath = "SpoqaHanSansNeo-Light.ttf"
        self.font = ImageFont.truetype(self.fontpath, 200)

    def callback(self, image):
        # rospy.loginfo('Image received...')
        # self.image = self.br.imgmsg_to_cv2(msg)
        detected_image, text = self.detect(image)
        print("detected result : ")
        print(text)
        return detected_image, text


    def load_model(self):
        self.car_m = torch.hub.load("ultralytics/yolov5", 'yolov5s', force_reload=True, skip_validation=True)
        self.lp_m = torch.hub.load('ultralytics/yolov5', 'custom', 'lp_det.pt')
        self.reader = easyocr.Reader(['en'], detect_network='craft', recog_network='best_acc', user_network_directory='lp_models/user_network', model_storage_directory='lp_models/models')
        self.car_m.classes = [2,3, 5, 7]
        rospy.loginfo('Finish Loading Model')

    def detect(self, im):
        to_draw = np.array(im)
        results = self.car_m(im)
        locs = results.xyxy[0]
        result_text = []

        if len(locs) == 0:
            
            result = self.lp_m(im)
            if len(result) == 0:
                result_text.append('검출된 차 없음')
            else:
                for rslt in result.xyxy[0]:
                    x2,y2,x3,y3 = [item1.cpu().detach().numpy().astype(np.int32) for item1 in rslt[:4]]
                    try:
                        extra_boxes = 0
                        im = cv2.cvtColor(cv2.resize(to_draw[y2 - extra_boxes:y3 + extra_boxes,x2 - extra_boxes:x3 + extra_boxes], (224,128)), cv2.COLOR_BGR2GRAY)
                        text = self.reader.recognize(im)[0][1]
                        result_text.append(text)
                    except Exception as e:
                        return cv2.resize(to_draw, (1280,1280)), ""
                    img_pil = Image.fromarray(to_draw)
                    draw = ImageDraw.Draw(img_pil)
                    draw.text( (x2-100,y2-300),  text, font=self.font, fill=(0,255,0))
                    to_draw = np.array(img_pil)
                    cv2.rectangle(to_draw, (x2.item(),y2.item()),(x3.item(),y3.item()),(0,255,0),10)

                return cv2.resize(to_draw, (1280,1280)), result_text
        
        for idx, item in enumerate(locs):
            x,y,x1,y1=[it.cpu().detach().numpy().astype(np.int32) for it in item[:4]]
            self.car_im = to_draw[y:y1, x:x1,:].copy()
            result = self.lp_m(Image.fromarray(self.car_im))
            
            if len(result) == 0:
                result_text.append("차는 검출됬으나, 번호판이 검출되지 않음")
            
            for rslt in result.xyxy[0]:
                x2,y2,x3,y3 = [item1.cpu().detach().numpy().astype(np.int32) for item1 in rslt[:4]]
                try:
                    extra_boxes = 0
                    im = cv2.cvtColor(cv2.resize(to_draw[y+y2 - extra_boxes:y+y3 + extra_boxes,x+x2 - extra_boxes:x+x3 + extra_boxes], (224,128)), cv2.COLOR_BGR2GRAY)
                    text = self.reader.recognize(im)[0][1]
                    result_text.append(text)
                except Exception as e:
                    return cv2.resize(to_draw, (1280,1280)), ""
                img_pil = Image.fromarray(to_draw)
                draw = ImageDraw.Draw(img_pil)
                draw.text( (x+x2-100,y+y2-300),  text, font=self.font, fill=(0,255,0))
                to_draw = np.array(img_pil)
                cv2.rectangle(to_draw, (x+x2,y+y2),(x+x3,y+y3),(0,255,0),10)
        
        return cv2.resize(to_draw, (1280,1280)), result_text
        
        
if __name__ == '__main__':


    rospy.init_node("init_klp_detection")
    rospy.loginfo("init_klp_detection")
    kLPDetection = KLPDetection()
    kLPDetection.load_model()
    cap = cv2.VideoCapture(0)
    cv2.namedWindow("result")

    while True:
        ret, frame = cap.read()
        if (frame.any() != None):
            detected_image, text = kLPDetection.callback(frame)
            if (text != []):
                cv2.imshow("result", detected_image)
            else:
                cv2.imshow("result", frame)
            if (cv2.waitKey(1) == 27):
                cap.release()
                cv2.destroyAllWindows()
                os._exit(1)
