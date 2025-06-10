import open3d as o3d
import numpy as np

def load_points_from_txt(file_path):
    """
    Charge un fichier contenant des points 3D au format x y z (un par ligne).
    
    Args:
        file_path (str): Chemin du fichier .txt contenant les points.
    
    Returns:
        numpy.ndarray: Tableau Numpy contenant les points 3D.
    """
    try:
        points = np.loadtxt(file_path, delimiter=' ')
        return points
    except Exception as e:
        print(f"Erreur lors de la lecture du fichier {file_path}: {e}")
        return None

def visualize_points(points):
    """
    Visualise les points 3D en utilisant Open3D.
    
    Args:
        points (numpy.ndarray): Tableau des points 3D.
    """
    if points is None or len(points) == 0:
        print("Aucun point à visualiser.")
        return
    
    # Convertir les points en un nuage de points Open3D
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    
    # Ajouter des couleurs aléatoires pour chaque point (facultatif)
    point_cloud.colors = o3d.utility.Vector3dVector(np.random.rand(points.shape[0], 3))
    
    # Visualiser le nuage de points
    o3d.visualization.draw_geometries([point_cloud], window_name="Nuage de points 3D")

if __name__ == "__main__":
    # Chemin vers le fichier contenant les points
    file_path = '3Dpoints/test_1'
    
    # Charger les points à partir du fichier
    points = load_points_from_txt(file_path)
    print(len(points))
    
    if points is not None:
        print(f"{len(points)} points chargés à partir de {file_path}")
        
        # Visualiser les points 3D
        visualize_points(points)
