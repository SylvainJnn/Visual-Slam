import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def plot_curves(my_data, ground_truth_data, data_tuto=None):
    # Séparation des données A et B en coordonnées x et z
    my_data_x = [point[0] for point in my_data]
    my_data_z = [point[1] for point in my_data]
    
    ground_truth_data_x = [point[0] for point in ground_truth_data]
    ground_truth_data_z = [point[1] for point in ground_truth_data]

    # # Tracé des courbes
    # plt.plot(my_data_z, my_data_x, label='My Data')
    # plt.plot(ground_truth_data_z, ground_truth_data_x, label='Ground Truth Data')

    # Tracé des points
    

    if(data_tuto != None):
        data_tuto_x = [point[0] for point in data_tuto]
        data_tuto_z = [point[1] for point in data_tuto]     
        plt.scatter(data_tuto_x, data_tuto_z, label='data tuto')
    plt.scatter(ground_truth_data_x, ground_truth_data_z, label='Ground Truth Data')
    plt.scatter(my_data_x, my_data_z, label='My Data')

    # Ajout de légendes et d'autres détails
    plt.xlabel('X')
    plt.ylabel('Z')
    plt.title('Comparaison entre My Data et Ground Truth Data')
    plt.legend()
    plt.grid(True)

    # Affichage du graphique
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
            # get 3rd and 9th element --> (x, z)
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
    my_data_seq1 = read_data("poses/my_poses_seq1.txt")
    ground_truth_data_seq1 = read_data("example/KITTI_sequence_1/poses.txt")
    data_tuto_seq1 = read_tuto_data("poses/tuto_poses_seq1.txt")
    # for data in ground_truth_data:
    #     print(data)
    # print(ground_truth_data)
    
    
    # plot_curves(my_data, ground_truth_data[1:])
    


    my_data_seq2 = read_data("poses/my_poses_seq2.txt")
    ground_truth_data_seq2 = read_data("example/KITTI_sequence_2/poses.txt")
    data_tuto_seq2 = read_tuto_data("poses/tuto_poses_seq2.txt")


    plot_curves(my_data_seq1, ground_truth_data_seq1, data_tuto_seq1)
    plot_curves(my_data_seq2, ground_truth_data_seq2, data_tuto_seq2)
    