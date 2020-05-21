import sys
import getopt
import serial
import numpy as np
import datetime as dt
import matplotlib.pyplot as plt

def main(argv):
    filename = 'data/' + argv[0]

    print("Acquisition time: " + argv[1] + "s.")
    print("Saving result to " + filename + ".npy.")

    # Setup serial port
    ser = serial.Serial()
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 115200

    ser.close()
    ser.open()

    if ser.is_open:
        print("Serial port opened.")

    # Acquisition time in µs
    T_acq = int(argv[1]) * 1e6

    # Data vectors (of unknown size a priori)
    time_pressure = []
    pressure = []
    time_flow = []
    flow = []
    time_peep = []
    peep = []

    # Throw first meaningless measurements + warm-up serial port (weird delay for
    # first readline() otherwise...)
    i = 0
    while i < 5:
        ser.flushInput()
        ser.readline()
        i += 1

    print("Ready to acquire. Press enter to start.")
    input()

    n = 0
    t0_pressure = 0
    t0_flow = 0
    while True:
        if n == 0:
            i = 0
            while i < 3:
                ser.flushInput()
                ser.readline()
                i += 1

        ser.flushInput()
        data_b = ser.readline()

        # Remove b'\x00' NUL character if any
        if data_b[0] == 0:
            data_b = data_b[1:]

        try:
            # Decode and remove \r\n
            data_str = data_b.decode().rstrip()

            # Separate data
            data = data_str.split(":")

            # Remove some weird cases randomly (and seldomly) appearing
            if len(data) == 4:
                print(data)

                # Record starting time
                if n == 0:
                    t0_pressure = int(data[0])
                    t0_flow = int(data[2])

                # Record data
                time_pressure.append(int(data[0]))
                pressure.append(int(data[1]))

                time_flow.append(int(data[2]))
                flow.append(int(data[3]))

                n += 1

                if int(data[2]) - t0_flow >= T_acq:
                    break

            if len(data) == 2:
                print(data)

                time_peep.append(int(data[0]))
                peep.append(int(data[1]))

        except UnicodeDecodeError as err:
            print("UnicodeDecodeError: ", err)
        except ValueError as err:
            print("ValueError: ", err)

    T_acq_eff = (int(data[2]) - t0_flow)/1e6
    print("Acquired " + str(n) + " samples in " + str(T_acq_eff) + "s.")

    # Close the serial port
    ser.close()
    print("Serial port closed.")

    time_pressure = np.array(time_pressure)
    time_flow = np.array(time_flow)
    time_peep = np.array(time_peep)

    # Set the starting time to 0, convert to seconds
    time_pressure = (time_pressure - t0_pressure)/1e6
    time_flow = (time_flow - t0_pressure)/1e6
    time_peep = (time_peep - t0_pressure)/1e6

    # Convert from 1/8cmH2O units to cmH2O
    pressure = np.array(pressure)/8

    # Convert from ml/min units to l/min
    flow = np.array(flow)/1000

    peep = np.array(peep)

    # Save data
    np.save(filename, [time_pressure, pressure, time_flow, flow, time_peep, peep])
    print("Saved in " + filename + ".npy.")

    # Plot data
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

if __name__ == '__main__':
    main(sys.argv[1:])
