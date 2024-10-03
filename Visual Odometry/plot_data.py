import matplotlib.pyplot as plt
import numpy as np


class data:
    def __init__(self, new_name, new_poses):
        self.name = new_name
        self.poses = new_poses


def plot_data(data):
    for d in data:
        dx = [points[0] for points in d.poses]
        dz = [points[1] for points in d.poses]
        plt.scatter(dx, dz, label=d.name)

    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('Visual Odometetry path poses')
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

# def read_tuto_data(filename):
#     data = []
#     with open(filename, 'r') as file:
#         for line in file:
#             # sepearent element of the line 
#             elements = line.split()
#             # get 1st and 2d element --> (x, z)
#             x = float(elements[0])  
#             z = float(elements[1])  
#             data.append([x, z])
#     return data


if __name__ == "__main__":
   
    ground_truth_seq1 = data('Ground truth data seq1', 
                             read_data("example/KITTI_sequence_1/poses.txt"))
    
    ground_truth_seq2 = data('Ground truth data seq2', 
                             read_data("example/KITTI_sequence_2/poses.txt"))

    # example_data_2D_2D = data('Example data', 
    #                     read_data("poses/seq2_2D_2D.txt"))
    
    # example_data_23_2D = data('Example data', 
    #                     read_data("poses/mono_3D_2D_my_poses_seq2.txt"))
    
    # plot_data([
    #     ground_truth_seq2, 
    #     example_data_2D_2D
    #     ])