import cv2
import numpy as np


class KalmanFilter:
    def __init__(self, initial_state, initial_estimate_error, process_noise, measurement_noise):
        self.state_estimate = initial_state
        self.estimate_error = initial_estimate_error
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.transition_matrix = np.eye(len(initial_state))
        self.measurement_matrix = np.eye(len(initial_state))

    # 状态预测
    def predict(self):
        predicted_state = np.dot(self.transition_matrix, self.state_estimate)
        predicted_error = np.dot(np.dot(self.transition_matrix, self.estimate_error), self.transition_matrix.T) + self.process_noise
        self.state_estimate = predicted_state
        self.estimate_error = predicted_error

    # 状态更新
    def update(self, measurement):
        kalman_gain = np.dot(np.dot(self.estimate_error, self.measurement_matrix.T), np.linalg.inv(np.dot(np.dot(self.measurement_matrix, self.estimate_error), self.measurement_matrix.T) + self.measurement_noise))
        self.state_estimate = self.state_estimate + np.dot(kalman_gain, (measurement - np.dot(self.measurement_matrix, self.state_estimate)))
        self.estimate_error = np.dot((np.eye(len(self.state_estimate)) - np.dot(kalman_gain, self.measurement_matrix)), self.estimate_error)


class GreenCapture:
    def __init__(self):
        self.lower_green = np.array([35, 100, 100])
        self.upper_green = np.array([85, 255, 255])

        # 卡尔曼滤波初始化
        self.kalman = KalmanFilter(initial_state=np.array([0, 0]), initial_estimate_error=np.array([[1, 0], [0, 1]]), process_noise=np.array([[0.01, 0], [0, 0.01]]), measurement_noise=np.array([[0.1, 0], [0, 0.1]]))

    def video_capture(self):
        self.cap = cv2.VideoCapture(0)

        # 检查摄像头是否开启
        if not self.cap.isOpened():
            print("Unable to open the camera")
            exit()

        while True:
            # 读取每一帧信息
            ret, self.image = self.cap.read()

            if not ret:
                print("Unable to read a video frame")
                break

            self.detect_green()

            cv2.imshow('Result', self.image)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def detect_green(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # 对绿色进行二值化
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        # 开运算
        kernel = np.ones([3, 3], np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        masked = cv2.bitwise_and(self.image, self.image, mask=mask)

        # 边缘检测
        canny = cv2.Canny(masked, 90, 155)

        # 轮廓检测
        contours, _ = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 轮廓检测后寻找面积最大值
        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            epsilon = 0.01 * cv2.arcLength(max_contour, True)
            approx = cv2.approxPolyDP(max_contour, epsilon, True)

            # 寻找质心，使用卡尔曼滤波
            M = cv2.moments(approx)
            if M['m00'] > 0:
                measured_position = np.array([int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])])
                self.kalman.predict()
                self.kalman.update(measured_position)
                estimated_position = self.kalman.state_estimate.astype(int)

                cv2.circle(self.image, (estimated_position[0], estimated_position[1]), 10, (0, 0, 255), -1)
                print('~x:', self.image.shape[1] / 2 - estimated_position[0], '~y:', self.image.shape[0] / 2 - estimated_position[1])


if __name__ == '__main__':
    run = GreenCapture()
    run.video_capture()
