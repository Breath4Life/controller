import sys
import getopt
import serial
import numpy as np
import datetime as dt
import matplotlib.pyplot as plt
import time
import plot_data

def blocking_readline(ser):
    r = b''
    while b'\n' not in r:
        r += ser.readline()
        time.sleep(0.02)

def setup_serial():
    # Setup serial port
    ser = serial.Serial()
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 115200
    ser.timeout = 0
    ser.close()
    ser.open()
    if ser.is_open:
        print("Serial port opened.")
    # Throw first meaningless measurements + warm-up serial port (weird delay for
    # first readline() otherwise...)
    for _ in range(5):
        ser.flushInput()
        blocking_readline(ser)
    return ser

class TimedDataSet:
    def __init__(self):
        self.t = np.zeros(0)
        self.v = np.zeros(0)
        self.used = 0
        self.normalization = None

    def add_data(self, times, values):
        times = list(times)
        values = list(values)
        assert len(times) == len(values)
        if self.t.shape[0] - self.used < len(times):
            new_size = max(2*self.t.shape[0], self.t.shape[0]+2*len(times))
            t = np.zeros(new_size)
            t[:self.used] = self.t
            self.t = t
            v = np.zeros(new_size)
            v[:self.used] = self.v
            self.v = v
        if self.normalization is None:
            times_n = np.array(times)
        else:
            times_n = np.array(times) - self.normalization
        self.t[self.used:self.used+len(times)] = times_n
        self.v[self.used:self.used+len(times)] = np.array(values)
        self.used += len(times)

    def times(self):
        return self.t[:self.used]

    def values(self):
        return self.v[:self.used]

    def t0(self):
        return self.t[0] if self.used else None

    def normalize_t(self, t0):
        assert self.normalization is None
        self.normalization = t0
        self.t[:self.used] -= t0


def normalize_data(data_map):
    # Set the starting time to 0, convert to seconds
    time_pressure = data_map['ip'].times()/1e6
    time_flow = data_map['fl'].times()/1e6
    time_peep = data_map['pe'].times()/1e6
    time_dp = data_map['dp'].times()/1e6

    # Convert from 1/8cmH2O units to cmH2O
    pressure = data_map['ip'].values()/8

    # Convert from ml/min units to l/min
    flow = data_map['fl'].values()/1000

    peep = data_map['pe'].values()
    dp = data_map['dp'].values()

    return time_pressure, pressure, time_flow, flow, time_peep, peep, time_dp, dp

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
            'ip': TimedDataSet(),
            'fl': TimedDataSet(),
            'pe': TimedDataSet(),
            'dp': TimedDataSet(),
            }
    
    plt.ion()
    plotter = plot_data.FPPPlotter()

    for _ in range(3):
        ser.flushInput()
        blocking_readline(ser)

    ts0 = 0
    ts_last = 0
    data_b = b''
    while ts_last-ts0 < t_acq*1e6:
        data_b += ser.readline()
        print('data:', data_b)

        if b'\n' in data_b:
            # Remove b'\x00' NUL character if any
            if data_b[0] == 0:
                print('Err \\0', data_b)
                data_b = data_b[1:]

            # Decode and remove \r\n
            data_str = data_b.decode('ascii').rstrip()

            if data_str.startswith(':'):
                try:
                    kind, ts, value = data_str[1:].split(':')
                    data_map[kind].add_data([int(ts)], [int(value)])
                    ts_last = int(ts)
                    if ts0 == 0:
                        ts0=ts_last
                        for tds in data_map.values():
                            tds.normalize_t(ts0)
                except Exception as e:
                    raise ValueError(repr(data_str)) from e
            data_b = b''
        else:
            plotter.upd_data(normalize_data(data_map))
            time.sleep(0.05)

    print("Acquired {} samples".format(sum(map(lambda x: x.used, data_map.values()))))

    # Close the serial port
    ser.close()
    print("Serial port closed.")

    time_pressure, pressure, time_flow, flow, time_peep, peep, time_dp, dp = normalize_data(data_map)

    # Save data
    np.save(filename, [time_pressure, pressure, time_flow, flow, time_peep, peep, time_dp, dp])
    print("Saved in " + filename + ".npy.")

    #plot_data.plot_flow_pressure_peep((time_pressure, pressure, time_flow, flow, time_peep, peep))
    plt.show(True)
    #plot_res(time_flow, time_pressure, flow, pressure)

if __name__ == '__main__':
    main(sys.argv[1:])
