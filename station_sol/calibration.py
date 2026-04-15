import cv2
import numpy as np
import glob
import os

# --- CONFIGURATION (Basée sur votre matériel) ---
CHECKERBOARD = (7, 7)  # Intersections internes (pour votre plateau 8x8 cases)
SQUARE_SIZE = 25       # Taille réelle d'une case en mm (facultatif, aide à l'échelle)
IMAGES_PATH = "/camera_calibration/*.jpg"  # Chemin vers vos images (ex: "images/*.jpg")

def calibrate_and_verify():
    # Critères de précision pour l'affinage des coins
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Préparation des points 3D (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp = objp * SQUARE_SIZE

    objpoints = [] # Points 3D dans le monde réel
    imgpoints = [] # Points 2d dans le plan de l'image

    images = glob.glob(IMAGES_PATH)
    if not images:
        print("Erreur : Aucune image trouvée. Vérifiez le chemin IMAGES_PATH.")
        return

    print(f"Analyse de {len(images)} images...")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 1. Détection des coins
        # On utilise des flags pour plus de robustesse face aux reflets
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            objpoints.append(objp)
            # 2. Affinage des coordonnées au sous-pixel près
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Visualisation immédiate
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            cv2.imshow('Detection en cours...', cv2.resize(img, (800, 600)))
            cv2.waitKey(100)
        else:
            print(f"Echec de détection pour : {fname}")

    cv2.destroyAllWindows()

    # 3. Calibration de la caméra
    print("\nCalcul des paramètres intrinsèques...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # 4. CALCUL DE L'ERREUR DE REPROJECTION (La preuve de qualité)
    # On reprojette les points 3D sur l'image et on compare avec les points détectés
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error

    rmse = total_error / len(objpoints)
    
    print("-" * 30)
    print(f"RESULTAT DE LA CALIBRATION :")
    print(f"Erreur moyenne (RMSE) : {rmse:.4f} pixels")
    
    if rmse < 0.5:
        print("Qualité : EXCELLENTE")
    elif rmse < 1.0:
        print("Qualité : BONNE")
    else:
        print("Qualité : MÉDIOCRE (Trop d'erreurs, recalibrez avec plus de lumière)")
    print("-" * 30)

    # 5. Sauvegarde
    np.savez("camera_params.npz", mtx=mtx, dist=dist)
    print("Paramètres sauvegardés dans 'camera_params.npz'")

    # 6. Test visuel d'Undistort (sur la dernière image)
    h, w = gray.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    
    # Recadrage selon la ROI pour enlever les bords noirs si besoin
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    
    cv2.imshow('Resultat final sans distorsion', cv2.resize(dst, (800, 600)))
    print("\nAppuyez sur une touche pour quitter.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrate_and_verify()