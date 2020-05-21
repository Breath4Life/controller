import csv
import sys
import numpy as np
import matplotlib.pyplot as plt

def main(argv):
    filename = 'data/' + argv[0]

    # Data from iFlow 200S and MPX7002DP
    data_sensor = np.load(filename + ".npy", allow_pickle=True)
    time_pressure, pressure, time_flow, flow, time_peep, peep = data_sensor

    # Plot data
    fig = plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(time_flow, flow, linewidth=1, marker='o', markersize=2,
             color='blue')
    for xc in time_peep:
        plt.axvline(x=xc, color='k', linestyle='--')
    plt.ylabel("Flow [l/min]")
    plt.xlabel("Time [s]")
    plt.title("Flow [l/min] (above) and pressure [cmH2O] (below) \n measured by MPX5010DP and iFlow 200S + MPXV7002DP")
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(time_pressure, pressure, linewidth=1, marker='o', markersize=2,
             color='red')
    for xc in time_peep:
        plt.axvline(x=xc, color='k', linestyle='--')

    plt.scatter(time_peep, peep, marker='o', color='k')
    plt.ylabel("Pressure [cmH2O]")
    plt.xlabel("Time [s]")
    plt.ylim([0, max(pressure) * 1.1])
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main(sys.argv[1:])
