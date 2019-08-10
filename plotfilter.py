from matplotlib import pyplot as plt
import numpy as np

class Filter:
    def __init__(self, params, size):
        self.params = params
        self.y = 0


    def apply(self, xinput):
        out = np.zeros(len(xinput))
        for i, x in enumerate(xinput):
            if (len(self.params) == 2 or (len(self.params) == 4 and np.abs(x) > np.abs(self.y))):
                self.y = self.params[0] * x + self.params[1] * self.y
            else:
                self.y = self.params[2] * x + self.params[3] * self.y
            out[i] = self.y
        return out

class WeirdFilter:
    def __init__(self, f1, f2):
        self.f1 = f1
        self.f2 = f2
        self.y1 = 0
        self.y2 = 0

    def apply(self, xinput):
        out1 = np.zeros(len(xinput))
        out2 = np.zeros(len(xinput))
        for i, x in enumerate(xinput):
            y1 = self.f1.params[0] * x + self.f1.params[1] * self.y1
            y2 = self.f2.params[0] * x + self.f2.params[1] * self.y2
            y1 = y1 + y2
            self.y1 = y1
            self.y2 = y2
            out1[i] = self.y1
            out2[i] = self.y2
        return out1, out2

class Gain:
    def __init__(self, params, kp, kd):
        self.params = params
        self.kp = kp
        self.kd = kd
        self.gain = 1
        self.y = 0
        self.err = 0

    def log_err(self, x):
        x = 1.000001 - x
        s = -1 if x > 0 else 1
        return s * np.log2(np.abs(x))

    def apply(self, xinput):
        gainOut = np.zeros(len(xinput))
        xOut = np.zeros(len(xinput))
        for i, x in enumerate(xinput):
            x = x * self.gain
            self.y = self.params[0] * x + self.params[1] * self.y

            err = self.log_err(1 - self.y)
            
            u = self.kp * err + self.kd * (err - self.err)
            self.err = err

            self.gain += u

            if self.gain > 1e8:
                self.gain = 1e8
            if self.gain < 1e-8:
                self.gain = 1e-8

            gainOut[i] = self.gain
            xOut[i] = x

        return gainOut, xOut


class Scaler:
    def __init__(self, params):
        self.params = params
        self.scale = 1
        self.vsh = 1

    def apply(self, xinput):
        out1 = np.zeros(len(xinput))
        out2 = np.zeros(len(xinput))
        for i, x in enumerate(xinput):
            sval = self.scale * (x-1)
            if sval < 0:
                sval = -sval
            if sval < self.vsh:
                self.vsh = self.params[0] * sval + self.params[1] * self.vsh
            else:
                self.vsh = self.params[2] * sval + self.params[3] * self.vsh

            if self.vsh < .001:
                self.vsh = .001
            self.scale = 1 / self.vsh

            out1[i] = self.scale
            out2[i] = self.vsh

        return out1, out2


def test1():
    size = 1024

    xin = np.zeros(size)
    for i in range(size):
        if (i % 256) < 16:
            xin[i] = 1

    f1 = Filter([.05, .95], size)
    f2 = Filter([-.0075, .9925], size)
    xout1 = f1.apply(xin)
    xout2 = f2.apply(xin)

    combine = xout1 + xout2

    plt.plot(xout1)
    plt.plot(xout2)
    plt.plot(combine)
    plt.show()

def test2():
    size = 1024

    xin = np.zeros(size)
    for i in range(size):
        if (i % 256) < 16:
            xin[i] = 1

    f1 = Filter([.05, .95], size)
    f2 = Filter([-.0075/5, .9925/5], size)

    wf = WeirdFilter(f1, f2)
    y1, y2 = wf.apply(xin)

    plt.plot(xin)
    plt.plot(y1)
    plt.plot(y2)
    plt.show()

def test3():
    size = 1024

    xin = np.zeros(size)
    for i in range(size):
        if (i % 256) < 16:
            xin[i] = .1

    f1 = Filter([.05, .95], size)
    f2 = Filter([-.0075, .9925], size)
    xout1 = f1.apply(xin)
    xout2 = f2.apply(xin)

    combine = xout1 + xout2

    sc = Scaler([.05, .95, .001, .999])
    scales, vsh = sc.apply(combine)

    # plt.plot(xin)
    p1, = plt.plot(combine, label="combine")
    p2, = plt.plot(scales, label="scales")
    p3, = plt.plot(combine * scales, label="combine*scales")

    plt.legend(handles=[p1,p2,p3])
    plt.show()

def test4():
    size = 1024*16

    xin = np.zeros(size)
    for i in range(size):
        if (i % 256) < 128:
            # print (i, i/256 )
            if (int(i / 256) % 4 == 1):
                xin[i] = 1
            else:
                xin[i] = .5
        else:
            xin[i] = .1

    gain = Gain([.005, .995], .001, .005)
    gainOut, xout = gain.apply(xin)

    # plt.plot(gainOut)
    # plt.show()

    f1 = Filter([.1, .9], size)
    f2 = Filter([-.0005, .9995, -.0075, .9925], size)
    xout1 = f1.apply(xout)
    xout2 = f2.apply(xout)

    combine = xout1 + xout2

    sc = Scaler([.001, .999, .0005, .9995])
    scales, vsh = sc.apply(combine)

    # plt.plot(xin)
    pg, = plt.plot(gainOut, label="gain")
    p0, = plt.plot(xout, label="x*gain")
    p1, = plt.plot(combine, label="combine")
    p2, = plt.plot(scales / 10, label="scales / 10")
    p5, = plt.plot(vsh, label="scale value filt")
    p3, = plt.plot(combine * scales, label="combine*scales")

    plt.legend(handles=[p0, p1,p2,p3, pg, p5])
    plt.show()

if __name__ == "__main__":
    # test1()
    # test2()
    # test3()
    test4()
