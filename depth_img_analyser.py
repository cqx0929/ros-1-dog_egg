import cv2
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


class DepthImageAnalyser(object):
    def __init__(
            self,
            save_img=True,
            show_2d=True,
            show_3d=False,
            measure_area=10000  # m*10^3
    ):
        # self.file_name: str = 'depth_image.txt'
        self.file_name: Path = Path('depth_image.txt')
        self.depth_img_save_path: Path = Path('img') / Path('depth_image.jpg')
        self.save_img: bool = save_img
        self.show_2d: bool = show_2d
        self.show_3d: bool = show_3d
        self.ls: np.array = self.__get_np_array()
        self.measure_area = self.__get_measure_area(measure_area)
        self.gray = self.__get_gray()
        self.gray255 = (self.__get_gray() * 255).astype(int)

    def show3d(self):
        x, y, z = self.__get_xyz()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, z, c='b', marker='.', linewidths=1)
        plt.show()

    def show2d(self):
        cv2.imshow('depth_img', self.gray)
        if cv2.waitKey(0) > 0:
            cv2.destroyAllWindows()

    def img_save(self):
        cv2.imwrite(str(self.depth_img_save_path), self.gray255)

    def __get_measure_area(self, measure_area) -> int:
        if 0 < measure_area < 10000:
            measure_area = measure_area
        else:
            measure_area = self.ls.max()
        return measure_area

    def __get_np_array(self) -> np.array:
        with open(self.file_name, 'r') as fp:
            content = fp.readlines()
        ls = [0] * len(content)
        for i, line in enumerate(content):
            line = line.rstrip('\n').split(' ')
            line = [int(i) for i in line]
            ls[i] = line
        return np.array(ls)

    def __get_gray(self):
        gray = self.ls / max(self.ls.flat)
        return gray

    def __get_gray255(self):
        gray255 = (self.ls / max(self.ls.flat)).astype(int)
        return gray255

    def __get_xyz(self):
        h, w = self.ls.shape[:-1] if len(self.ls.shape) > 2 else self.ls.shape
        xcs = []
        ycs = []
        zcs = []
        for i in range(h):
            for j in range(w):
                ycs.append(i)
                xcs.append(j)
                z = self.__get_real_z(self.ls[i, j])
                zcs.append(z)
        return xcs, ycs, zcs

    @staticmethod
    def __get_real_z(z):
        return z / 1000

    def go(self):
        if self.show_2d and not self.show_3d:
            self.show2d()
        if not self.show_2d and self.show_3d:
            self.show3d()
        if self.save_img:
            self.img_save()
        if self.show_2d and self.show_3d:
            self.show2d()
            self.show3d()


if __name__ == '__main__':
    gd = DepthImageAnalyser(
        save_img=True,
        show_2d=True,
        show_3d=True,
        measure_area=10000
    )
    gd.go()
    exit()
