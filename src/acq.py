import sys
import getopt
import serial
import numpy as np
import datetime as dt
import matplotlib.pyplot as plt
import time
import plot_data

def setup_serial():
    # Setup serial port
    ser = serial.Serial()
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 115200
    ser.close()
    ser.open()
    if ser.is_open:
        print("Serial port opened.")
    # Throw first meaningless measurements + warm-up serial port (weird delay for
    # first readline() otherwise...)
    for _ in range(5):
        ser.flushInput()
        ser.readline()
    return ser

def normalize_data(data_map):
    t0 = min(x[0][0] for x in data_map.values() if x[0])
    time_pressure = np.array(data_map['ip'][0])
    time_flow = np.array(data_map['fl'][0])
    time_peep = np.array(data_map['pe'][0])

    # Set the starting time to 0, convert to seconds
    time_pressure = (time_pressure - t0)/1e6
    time_flow = (time_flow - t0)/1e6
    time_peep = (time_peep - t0)/1e6

    # Convert from 1/8cmH2O units to cmH2O
    pressure = np.array(data_map['ip'][1])/8

    # Convert from ml/min units to l/min
    flow = np.array(data_map['fl'][1])/1000

    peep = np.array(data_map['pe'][1])

    return time_pressure, pressure, time_flow, flow, time_peep, peep

def main(argv):
    filename = 'data/' + argv[0]

    print("Acquisition time: " + argv[1] + "s.")
    print("Saving result to " + filename + ".npy.")

    ser = setup_serial()
    print("Ready to acquire. Press enter to start.")
    input()

    # Acquisition time in s
    t_acq = int(argv[1])

    # Data vectors (of unknown size a priori)
    data_map = {
            'ip': ([], []),
            'fl': ([], []),
            'pe': ([], []),
            }
    
    plt.ion()
    plotter = plot_data.FPPPlotter()

    for _ in range(3):
        ser.flushInput()
        ser.readline()

    ts0 = 0
    ts_last = 0
    while ts_last-ts0 < t_acq*1e6:
        data_b = ser.readline()

        # Remove b'\x00' NUL character if any
        if data_b[0] == 0:
            print('Err \\0', data_b)
            data_b = data_b[1:]

        # Decode and remove \r\n
        data_str = data_b.decode('ascii').rstrip()

        if not data_str.startswith(':'):
            continue
        try:
            kind, ts, value = data_str[1:].split(':')
            data_map[kind][0].append(int(ts))
            data_map[kind][1].append(int(value))
            ts_last = int(ts)
            if ts0 == 0:
                ts0=ts_last
        except Exception as e:
            raise ValueError(data_str) from e

        plotter.upd_data(normalize_data(data_map))

    print("Acquired {} samples".format(sum(map(len, data_map.values()))))

    # Close the serial port
    ser.close()
    print("Serial port closed.")

    time_pressure, pressure, time_flow, flow, time_peep, peep = normalize_data(data_map)

    # Save data
    np.save(filename, [time_pressure, pressure, time_flow, flow, time_peep, peep])
    print("Saved in " + filename + ".npy.")

    #plot_data.plot_flow_pressure_peep((time_pressure, pressure, time_flow, flow, time_peep, peep))
    plt.show(True)
    #plot_res(time_flow, time_pressure, flow, pressure)

if __name__ == '__main__':
    main(sys.argv[1:])
