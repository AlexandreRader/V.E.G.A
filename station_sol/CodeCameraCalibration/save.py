import numpy as np

# 1. Préparation des données (Vos résultats)
camera_matrix = np.array([
    [1.46836115e+03, 0.00000000e+00, 6.57038765e+02],
    [0.00000000e+00, 1.46521159e+03, 8.90282289e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

dist_coeffs = np.array([
    [-0.17553074, 0.38779669, -0.00278653, -0.00300346, -0.35842445]
])

# 2. Sauvegarde dans le fichier .npz
# Note : on utilise les noms de clés 'camera_matrix' et 'dist_coeffs' 
# car c'est ce que votre programme principal attend.
np.savez('camera_calibration.npz', 
         camera_matrix=camera_matrix, 
         dist_coeffs=dist_coeffs)

print("Le fichier 'camera_calibration.npz' a été créé avec succès.")