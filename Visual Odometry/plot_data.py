import matplotlib.pyplot as plt
import numpy as np


class data:
    def __init__(self, new_name, new_poses):
        self.name = new_name
        self.poses = new_poses


def plot_data(data, line=True, count=True):
    for d in data:
        dx = [points[0] for points in d.poses]
        dz = [points[1] for points in d.poses]

        if(line):
            plt.plot(dx, dz, label=d.name)
        else:
            plt.scatter(dx, dz, label=d.name)

        # Ajouter des numéros pour chaque point
        if(count):
            if(d.name != "Ground truth data seq2"):
                for i, (x, z) in enumerate(zip(dx, dz)):
                    plt.text(x, z, str(i+1), fontsize=9, ha='right', va='bottom')  # Affichage du numéro (i+1)


    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('Visual Odometetry path poses')
    plt.legend()
    plt.grid(True)

    # set 1:1 scalre size
    # plt.gca().set_aspect('equal', adjustable='box')
    
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
    #     ground_truth_seq1,
    #     example_data_2D_2D
    #     ],
    #     line=False,
    #     count=True)
