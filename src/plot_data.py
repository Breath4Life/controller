import csv
import sys
import numpy as np
import matplotlib.pyplot as plt

class FPPPlotter:
    def __init__(self, data=None, flush=False):
        # Plot data
        self.fig = plt.figure()
        self.ax_flow = self.fig.add_subplot(2, 1, 1)
        self.ax_flow.set_title("Flow (iFlow 200S + MPXV7002DP)")
        self.ax_flow.set_xlabel("Time [s]")
        self.ax_flow.set_ylabel("Flow [l/min]")
        self.ax_flow.grid(True)
        self.line_flow, = self.ax_flow.plot([0, 1], [0, 1], linewidth=1, marker='o', markersize=2, color='blue')

        self.ax_p = self.fig.add_subplot(2, 1, 2)
        self.ax_p.set_title("Pressure (MPX5010DP)")
        self.ax_p.set_xlabel("Time [s]")
        self.ax_p.set_ylabel("Pressure [cmH2O]")
        self.ax_p.grid(True)
        #self.line_p,  = self.ax_p.plot([0], [0], linewidth=1, marker='o', markersize=2, color='red')
        #self.scatter_peep = self.ax_p.scatter([], [], marker='o', color='k')

        if data is not None:
            self.upd_data(data, flush)

    def upd_data(self, data_sensor, flush=True):
        time_pressure, pressure, time_flow, flow, time_peep, peep = data_sensor

        self.line_flow.set_xdata(time_flow)
        self.line_flow.set_ydata(flow)
        self.ax_flow.relim()
        #for xc in time_peep:
        #    self.ax_flow.axvline(x=xc, color='k', linestyle='--')

        self.line_p,  = self.ax_p.plot(time_pressure, pressure, linewidth=1, marker='o', markersize=2, color='red')
        #self.line_p.set_data(time_pressure, pressure)
        #for xc in time_peep:
        #    plt.axvline(x=xc, color='k', linestyle='--')
        #self.scatter_peep = self.ax_p.scatter(time_peep, peep, marker='o', color='k')
        #self.scatter_peep.set_data(time_peep, peep)
        if flush:
            plt.draw()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()


def plot_flow_pressure_peep(data_sensor):
    FPPPlotter(data_sensor)

def plot_res(time_flow, time_pressure, flow, pressure):
    fig = plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(time_flow, flow, linewidth=3, marker='o', markersize=2,
             color='blue')
    plt.ylabel("Flow [l/min]")
    plt.xlabel("Time [s]")
    plt.title("Flow [l/min] (above) and pressure [cmH2O] (below) \n measured by MPX5010DP and iFlow 200S + MPXV7002DP")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(time_pressure, pressure, linewidth=3, marker='o', markersize=2,
             color='red')
    plt.ylabel("Pressure [cmH2O]")
    plt.xlabel("Time [s]")
    plt.grid(True)

    plt.show()

def main(argv):
    filename = 'data/' + argv[0]

    # Data from iFlow 200S and MPX7002DP
    data_sensor = np.load(filename + ".npy", allow_pickle=True)

    plot_flow_pressure_peep(data_sensor)

    plt.show()

if __name__ == "__main__":
    main(sys.argv[1:])
