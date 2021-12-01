import cv2
import os
from itertools import compress


def main():
    images = os.listdir('/home/mtn/agrobot_ws/src/agrobot/agrobot_rl/tmp/')
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    for file in [im for im in images if im.endswith('.png')]:
        fp = os.path.join('/home/mtn/agrobot_ws/src/agrobot/agrobot_rl/tmp/', file)
        print(fp)
        image = cv2.imread(fp)

        cv2.namedWindow('map', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('map', 800, 800)
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_dill = cv2.dilate(img_gray, kernel)

        ret, im = cv2.threshold(img_dill, 127, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(im, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print(hierarchy)
        has_inner = [x > 0 for x in [el[3] for el in hierarchy[0]]]
        print(has_inner)
        area = [cv2.contourArea(cnt) for cnt in contours]
        print(sum(area[1:]))
        # for cnt in range(len(contours)):
        #     print(cv2.contourArea(contours[cnt]))
        print(any([x > 100 for x in list(compress(area, has_inner))]))
        img = cv2.drawContours(image, contours, -1, (0, 255, 75), 1)
        cv2.imshow('map', img)
        cv2.waitKey()


if __name__ == '__main__':
    main()
