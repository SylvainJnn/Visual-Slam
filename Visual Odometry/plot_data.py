import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


class data:
    def __init__(self, new_name, new_pointclouds):
        self.name = new_name
        self.pointclouds = new_pointclouds


def plot_data(data):
    for d in data:
        dx = [points[0] for points in d.pointclouds]
        dz = [points[1] for points in d.pointclouds]
        plt.scatter(dx, dz, label=d.name)

    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('Visual Odometetry point clouds')
    plt.legend()
    plt.grid(True)
    
    plt.show()


def read_data(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file:
            # sepearent element of the line 
            elements = line.split()
            # get 3rd and 9th element --> (x, z)
            x = float(elements[3])  # Le 4ème élément (index 3)
            z = float(elements[11])  # Le 12ème élément (index 11)
            data.append([x, z])
    return data

def read_tuto_data(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file:
            # sepearent element of the line 
            elements = line.split()
            # get 4rd and 11th element --> (x, z)
            x = float(elements[0])  # Le 4ème élément (index 3)
            z = float(elements[1])  # Le 12ème élément (index 11)
            data.append([x, z])
    return data

def read_data_pandas(filename):
    # read file
    df = pd.read_csv(filename, delim_whitespace=True, header=None)
    # get x and z
    data = df.iloc[:, [4, 11]].values.tolist()
    return data


if __name__ == "__main__":
    my_data_seq1 = data("my_data_seq1", 
                         read_data("poses/my_poses_seq1.txt")) 
    ground_truth_data_seq1 = data("ground_truth_data_seq1",
                                   read_data("example/KITTI_sequence_1/poses.txt"))
    data_tuto_seq1 = data("data_tuto_seq1",
                           read_tuto_data("poses/tuto_poses_seq1.txt"))
    
    data = [my_data_seq1,
            ground_truth_data_seq1,
            data_tuto_seq1]
    
    plot_data(data)