import csv
import matplotlib.pyplot as plt

def load_data_from_csv(file_path):
    data = [[] for _ in range(9)]

    with open(file_path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            for i in range(9):
                # print(row)
                data[i].append(float(row[i]))

    return data

def plot_data(data):
    x = range(len(data[0]))
    labels = ['payload x', 'payload y', 'payload z', 'ada_roll', 'ada_pitch', 'ada_yaw', 'ada_z', 'ada_x', 'ada_y']

    # labels = ['payload x', 'payload y', 'sigma_roll', "sigma_pitch"]
    # labels = ['payload x', 'payload y', 'payload z']

    # for i in range(9):
    #     plt.plot(x, data[i], label=labels[i])
    i=3
    plt.plot(x, data[i], label=labels[i])
    i=4
    plt.plot(x, data[i], label=labels[i])
    i=2
    # plt.plot(x, data[i], label=labels[i])
    # i=7
    # plt.plot(x, data[i], label=labels[i])
    # i=8
    # plt.plot(x, data[i], label=labels[i])
    # i=6
    # plt.plot(x, data[i], label=labels[i])

    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()
    plt.show()

file_path = "/home/seunghyun/wo_paylaod_with_adaptive_cmd.csv"

data = load_data_from_csv(file_path)

# 데이터 플롯하기
plot_data(data)