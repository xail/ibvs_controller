import matplotlib.pyplot as plt
from os.path import expanduser
import numpy as np
import scipy.io

home = expanduser("~")


def plotVel(v, t, name='v', folder=home + '/resources/', eps=True, linear=True, zeroStartTime=True):
    if len(v) > 0:
        if len(v) != len(t):
            t = t[0, len(v)]
        if zeroStartTime:
            t = [i - t[0] for i in t]
        plt.plot(t, v, color='black')
        plt.gca().set_xlabel('t, с')
        if linear:
            plt.gca().set_ylabel('v, м/с')
        else:
            plt.gca().set_ylabel('$/omega$, рад/с')
        plt.gca().set_xlim(left=min(t), right=max(t))
        plt.gca().set_ylim(bottom=min(v), top=max(v))
        if eps:
            plt.savefig(folder + name + '.eps')
        else:
            plt.savefig(folder + name + '.pdf')
        plt.close()


def save_matlab(list_of_input, list_of_output, time, name='mtlb_data', folder=home + '/resources/matlab/'):
    input1 = np.asarray(list_of_input[0])
    input2 = np.asarray(list_of_input[1])
    output1 = np.asarray(list_of_output[0])
    output2 = np.asarray(list_of_output[1])
    t = np.asarray(time)
    scipy.io.savemat(folder + name + '.mat', mdict={'input1': input1, 'input2': input2, 'output1': output1, 'output2': output2, 'time': t})
