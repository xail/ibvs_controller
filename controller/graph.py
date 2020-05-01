import matplotlib.pyplot as plt
from os.path import expanduser

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