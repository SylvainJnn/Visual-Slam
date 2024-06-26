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
            # get 4rd and 12th element --> (x, z)
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
            # get 1st and 2d element --> (x, z)
            x = float(elements[0])  
            z = float(elements[1])  
            data.append([x, z])
    return data

def read_data_pandas(filename):
    # read file
    df = pd.read_csv(filename, delim_whitespace=True, header=None)
    # get x and z
    data = df.iloc[:, [4, 11]].values.tolist()
    return data

