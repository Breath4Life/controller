import csv
import sys
import numpy as np
import matplotlib.pyplot as plt

class FPPPlotter:
    def __init__(self, data=None, flush=False, lookback=20):
        self.lookback = lookback
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
        self.line_p, = self.ax_p.plot([0], [0], linewidth=1, marker='o', markersize=2, color='red')
        self.line_peep, = self.ax_p.plot([], [], 'x', color='k')
        self.line_dp, = self.ax_p.plot([], [], 'd', color='orange')

        if data is not None:
            self.upd_data(data, flush)

    def upd_data(self, data_sensor, flush=True):
        time_pressure, pressure, time_flow, flow, time_peep, peep, time_dp, dp = data_sensor
        times = (time_pressure, time_flow, time_peep, time_dp)

        if self.lookback is not None:
            last_sample = max((x[-1] for x in times if x.shape[0]), default=0)
            first_sample = min((x[0] for x in times if x.shape[0]), default=0)
            time_range_start = max(first_sample, last_sample-self.lookback)
            time_range_end = time_range_start + self.lookback

        self.line_flow.set_data(time_flow, flow)
        self.ax_flow.relim()
        if self.lookback is not None:
            self.ax_flow.set_xlim(time_range_start, time_range_end)
        self.ax_flow.autoscale_view()
        #for xc in time_peep:
        #    self.ax_flow.axvline(x=xc, color='k', linestyle='--')

        self.line_p.set_data(time_pressure, pressure)
        self.line_peep.set_data(time_peep, peep)
        self.line_dp.set_data(time_dp, dp)
        self.ax_p.relim()
        if self.lookback is not None:
            self.ax_p.set_xlim(time_range_start, time_range_end)
        self.ax_p.autoscale_view()
        #for xc in time_peep:
        #    plt.axvline(x=xc, color='k', linestyle='--')
        if flush:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()


def plot_flow_pressure_peep(data_sensor):
    FPPPlotter(data_sensor, lookback=None)

def main(argv):
    filename = 'data/' + argv[0]

    # Data from iFlow 200S and MPX7002DP
    data_sensor = np.load(filename + ".npy", allow_pickle=True)

    plot_flow_pressure_peep(data_sensor)

    plt.show()

if __name__ == "__main__":
    main(sys.argv[1:])
