import matplotlib.pyplot as plt
from os.path import expanduser

home = expanduser("~")


def plotVel_lin(v, t, name='v', folder=home + '/resources/', eps=True):
    if len(v) > 0:
        if len(v) != len(t):
            t = t[0, len(v)]
        plt.plot(t, v, color='black')
        plt.gca().set_xlabel('t, с')
        plt.gca().set_ylabel('v, м/с')
        plt.gca().set_xlim(left=min(t), right=max(t))
        plt.gca().set_ylim(bottom=min(v), top=max(v))
        if eps:
            plt.savefig(folder + name + '.eps')
        else:
            plt.savefig(folder + name + '.pdf')
        plt.close()


def plotVel_ang(w, t, name='omega', folder=home + '/resources/', eps=True):
    if len(w) > 0:
        if len(w) != len(t):
            t = t[0, len(w)]
        plt.plot(t, w, color='black')
        plt.gca().set_xlabel('t, с')
        plt.gca().set_ylabel('omega, рад/с')
        plt.gca().set_xlim(left=min(t), right=max(t))
        plt.gca().set_ylim(bottom=min(w), top=max(w))
        if eps:
            plt.savefig(folder + name + '.eps')
        else:
            plt.savefig(folder + name + '.pdf')
        plt.close()