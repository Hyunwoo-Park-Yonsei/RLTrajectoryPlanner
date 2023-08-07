import matplotlib.pyplot as plt
import numpy as np

class Visualizer:
    def __init__(self):
        pass

    def polynomial(self, params, T, step = 0.25):
        # 시간에 따른건지, 위치에 따른건지 찾아보기
        x = np.arange(0, T, step)

        y = params[0] + params[1] * x + params[2] * (x ** 2) + params[3] * (x ** 3) + params[4] * (x ** 4) + params[5] * (x ** 5)
        plt.plot(x, y)
        plt.show()

