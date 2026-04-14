import queue

import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import os
import glob
import heapq # Ajout pour l'algorithme A*




# --- CONFIGURATION PHYSIQUE DE LA PISTE ---
MAP_WIDTH_M = 4.0   
MAP_HEIGHT_M = 6.0  
CELL_SIZE_CM = 10.0 

# --- CONFIGURATION CALIBRATION ---
CHESSBOARD_SIZE = (7, 7)    # coins intérieurs du motif de calibration
SQUARE_SIZE = 1.0           # taille arbitraire des cases, unité relative

# --- CONFIGURATION TRAITEMENT ---
ROBOT_WIDTH_CM = 30.0 
SAFE_MARGIN_CM = 40.0 

# --- CALCULS CONSTANTS ---
GRID_W = int((MAP_WIDTH_M * 100) / CELL_SIZE_CM)  
GRID_H = int((MAP_HEIGHT_M * 100) / CELL_SIZE_CM) 
SCALE_UI = 10 

class CostmapApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Station Sol - Éditeur de Mission VEGA SC317 (Planner Intégré)")
        self.root.geometry("1400x850") 
        
        self.original_img = None
        self.warped_img = None
        self.costmap_grid = None 
        self.current_path = [] # NOUVEAU : Stockage de la trajectoire
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_file = None
        
        self.points_source = [] 
        self.canvas_points = [] 
        self.start_pos = None 
        self.end_pos = None   
        
        self.photo_orig = None
        self.photo_cost = None
        
        # --- Variables Outils ---
        self.right_tool_mode = tk.StringVar(value="none")
        self.brush_size = tk.IntVar(value=3)
        self.brush_cost = tk.IntVar(value=255) 
        self.overlay_alpha = tk.IntVar(value=70) 
        
        # Variables Canny & Contours
        self.canny_low = tk.IntVar(value=50)
        self.canny_high = tk.IntVar(value=150)
        self.min_area = tk.IntVar(value=500) 
        
        self.setup_ui()

    def setup_ui(self):
        # === BARRE DU HAUT ===
        btn_frame = tk.Frame(self.root, bg="#f0f0f0")
        btn_frame.pack(side=tk.TOP, fill=tk.X, pady=10)
        btn_style = {"font": ("Arial", 10, "bold"), "fg": "white", "padx": 10, "pady": 5}

        tk.Button(btn_frame, text="1. Charger l'image", command=self.load_image, bg="#4CAF50", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="Calibrer Caméra", command=self.calibrate_camera, bg="#607D8B", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="Charger Calib.", command=self.load_calibration, bg="#795548", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="2. Redresser (Perspective)", command=self.correct_perspective, bg="#9C27B0", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="3. Générer Costmap", command=self.process_image, bg="#2196F3", **btn_style).pack(side=tk.LEFT, padx=10)
        # NOUVEAU BOUTON !
        tk.Button(btn_frame, text="4. Calculer Trajectoire", command=self.run_pathfinding, bg="#e74c3c", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="5. Exporter Mission C++", command=self.export_code, bg="#FF9800", **btn_style).pack(side=tk.RIGHT, padx=20)
        
        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=20)
        
        # === GAUCHE : Caméra & Outils Canny (Restauré à 100%) ===
        orig_frame = tk.Frame(self.canvas_frame)
        orig_frame.pack(side=tk.LEFT, padx=10)
        tk.Label(orig_frame, text="Caméra Originale", font=("Arial", 11, "bold")).pack()
        
        self.canvas_orig = tk.Canvas(orig_frame, width=500, height=500, bg="#ddd", borderwidth=2, relief="groove", cursor="crosshair")
        self.canvas_orig.pack()
        self.canvas_orig.bind("<Button-1>", self.on_canvas_orig_click)

        left_toolbar = tk.Frame(orig_frame, pady=5)
        left_toolbar.pack(fill=tk.X)
        tk.Button(left_toolbar, text="🔄 Effacer Cadre", command=self.reset_points, font=("Arial", 9)).pack(side=tk.LEFT, padx=5)
        
        param_frame = tk.Frame(orig_frame, pady=5, bd=1, relief="ridge")
        param_frame.pack(fill=tk.X, pady=5)
        tk.Label(param_frame, text="⚙️ Réglages Détection :", font=("Arial", 9, "bold")).pack(anchor="w", padx=5)
        
        row1 = tk.Frame(param_frame); row1.pack(fill=tk.X, padx=5)
        tk.Label(row1, text="Sensibilité:", width=12, anchor="e").pack(side=tk.LEFT)
        tk.Scale(row1, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.canny_low, showvalue=1).pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        row2 = tk.Frame(param_frame); row2.pack(fill=tk.X, padx=5)
        tk.Label(row2, text="Contraste:", width=12, anchor="e").pack(side=tk.LEFT)
        tk.Scale(row2, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.canny_high, showvalue=1).pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        row3 = tk.Frame(param_frame); row3.pack(fill=tk.X, padx=5)
        tk.Label(row3, text="Filtre:", width=12, anchor="e").pack(side=tk.LEFT)
        tk.Scale(row3, from_=10, to=5000, orient=tk.HORIZONTAL, variable=self.min_area, showvalue=1, resolution=50).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # === DROITE : Costmap & Éditeur (Restauré à 100%) ===
        cost_frame = tk.Frame(self.canvas_frame)
        cost_frame.pack(side=tk.LEFT, padx=20)
        tk.Label(cost_frame, text=f"Costmap avec Overlay", font=("Arial", 11, "bold")).pack()
        
        self.canvas_cost = tk.Canvas(cost_frame, width=GRID_W*SCALE_UI, height=GRID_H*SCALE_UI, bg="black", borderwidth=2, relief="groove", cursor="target")
        self.canvas_cost.pack()
        self.canvas_cost.bind("<Button-1>", self.on_costmap_mouse)
        self.canvas_cost.bind("<B1-Motion>", self.on_costmap_mouse)

        right_toolbar = tk.Frame(cost_frame, pady=5)
        right_toolbar.pack(fill=tk.X)
        tk.Radiobutton(right_toolbar, text="Sélection", variable=self.right_tool_mode, value="none").pack(side=tk.LEFT)
        tk.Radiobutton(right_toolbar, text="🖌️ Édition", variable=self.right_tool_mode, value="brush", font=("Arial", 9, "bold")).pack(side=tk.LEFT)
        tk.Label(right_toolbar, text="| Taille:").pack(side=tk.LEFT)
        tk.Scale(right_toolbar, from_=1, to=10, orient=tk.HORIZONTAL, variable=self.brush_size, length=50, showvalue=0).pack(side=tk.LEFT)
        tk.Label(right_toolbar, text=" Intensité:").pack(side=tk.LEFT)
        tk.Scale(right_toolbar, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.brush_cost, length=70, tickinterval=255).pack(side=tk.LEFT)
        
        bottom_right = tk.Frame(cost_frame, pady=5)
        bottom_right.pack(fill=tk.X)
        tk.Radiobutton(bottom_right, text="🟢 A", variable=self.right_tool_mode, value="start", fg="green").pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(bottom_right, text="🔴 B", variable=self.right_tool_mode, value="end", fg="red").pack(side=tk.LEFT)
        tk.Label(bottom_right, text="  |  Transparence Costmap :").pack(side=tk.LEFT, padx=(10, 0))
        tk.Scale(bottom_right, from_=0, to=100, orient=tk.HORIZONTAL, variable=self.overlay_alpha, length=120, showvalue=0, command=lambda v: self.update_costmap_canvas()).pack(side=tk.LEFT)

        self.lbl_status = tk.Label(self.root, text="Statut : Prêt.", font=("Arial", 9), fg="#777")
        self.lbl_status.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

    # === METHODES IMAGE (Inchangées) ===
    def get_display_image(self, img):
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            return cv2.undistort(img, self.camera_matrix, self.dist_coeffs)
        return img

    def redraw_original_image(self):
        if self.original_img is None:
            return
        img_disp = self.fit_image_to_box(self.get_display_image(self.original_img))
        self.photo_orig = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(img_disp, cv2.COLOR_BGR2RGB)))
        self.canvas_orig.delete("all")
        self.canvas_orig.create_image(250, 250, image=self.photo_orig, anchor=tk.CENTER)
        if self.points_source:
            self.reset_points()

    def calibrate_camera(self):
        image_paths = list(filedialog.askopenfilenames(title="Sélectionner images de calibration", filetypes=[("Images", "*.jpg *.jpeg *.png *.JPG *.JPEG *.PNG")]))
        folder = None
        if not image_paths:
            folder = filedialog.askdirectory(title="Choisir dossier images de calibration")
            if not folder:
                return
            image_paths = sorted(
                glob.glob(os.path.join(folder, "*.jpg")) +
                glob.glob(os.path.join(folder, "*.jpeg")) +
                glob.glob(os.path.join(folder, "*.png")) +
                glob.glob(os.path.join(folder, "*.JPG")) +
                glob.glob(os.path.join(folder, "*.JPEG")) +
                glob.glob(os.path.join(folder, "*.PNG"))
            )
        elif image_paths:
            # If individual files were selected, use the directory of the first file
            folder = os.path.dirname(image_paths[0])

        if len(image_paths) < 3:
            messagebox.showerror("Erreur", f"Au moins 3 images de calibration sont nécessaires. Images trouvées : {len(image_paths)}")
            return

        pattern = CHESSBOARD_SIZE
        objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * SQUARE_SIZE

        objpoints = []
        imgpoints = []
        valid_images = 0
        first_valid_img = None
        last_valid_img = None

        for idx, path in enumerate(image_paths):
            stream = np.fromfile(path, dtype=np.uint8)
            img = cv2.imdecode(stream, cv2.IMREAD_COLOR)
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern, 
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)
            if found:
                valid_images += 1
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                objpoints.append(objp)
                imgpoints.append(corners2)
                
                # VISUALISATION : Créer une fenêtre pour voir la détection
                debug_img = img.copy()
                cv2.drawChessboardCorners(debug_img, pattern, corners2, found)
                cv2.imshow(f"Detection - {os.path.basename(path)}", cv2.resize(debug_img, (800, 600)))
                cv2.waitKey(500) # Attend 0.5s par image

                if first_valid_img is None:
                    first_valid_img = cv2.drawChessboardCorners(img.copy(), pattern, corners2, found)
                last_valid_img = cv2.drawChessboardCorners(img.copy(), pattern, corners2, found)

        if valid_images < 3:
            help_text = ("Aucun damier n'a pu être détecté. Suggestions :\n\n"
                        "1. Utilisez un damier PAPIER ou PLASTIQUE (pas de verre)\n"
                        "2. Améliorez l'éclairage (évitez reflets et ombres)\n"
                        "3. Assurez-vous qu'aucune partie du damier n'est coupée\n"
                        "4. Variez les angles et distances de prise de vue\n"
                        "5. Vérifiez la taille du damier : 9x6 coins intérieurs\n\n"
                        f"Images trouvées : {len(image_paths)}\n"
                        f"Images avec damier détecté : {valid_images}")
            messagebox.showerror("Erreur calibration", help_text)
            return
        
        if first_valid_img is not None:
            disp_img = cv2.resize(first_valid_img, (400, 300))
            cv2.imshow("Damier détecté (1ère image valide)", disp_img)
            cv2.waitKey(2000)
            cv2.destroyAllWindows()

        ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        self.camera_matrix = mtx
        self.dist_coeffs = dist
        self.calibration_file = os.path.join(folder, "camera_calibration.npz")
        np.savez(self.calibration_file, camera_matrix=mtx, dist_coeffs=dist)

        messagebox.showinfo("Calibration réussie", f"Calibration enregistrée dans :\n{self.calibration_file}\nErreur RMS = {ret:.4f}")
        self.lbl_status.config(text="Calibration chargée.")
        if self.original_img is not None:
            self.redraw_original_image()

    def load_calibration(self):
        path = filedialog.askopenfilename(filetypes=[("NumPy file", "*.npz")], title="Charger calibration camera")
        if not path:
            return
        try:
            data = np.load(path)
            self.camera_matrix = data["camera_matrix"]
            self.dist_coeffs = data["dist_coeffs"]
            self.calibration_file = path
            self.lbl_status.config(text=f"Calibration chargée : {os.path.basename(path)}")
            if self.original_img is not None:
                self.redraw_original_image()
        except Exception as e:
            messagebox.showerror("Erreur", f"Impossible de charger la calibration :\n{e}")

    def fit_image_to_box(self, img, box_size=500):
        h, w = img.shape[:2]
        scale = box_size / max(h, w)
        return cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)

    def load_image(self):
        path = filedialog.askopenfilename(filetypes=[("Images", "*.jpg *.jpeg *.png")])
        if path:
            stream = np.fromfile(path, dtype=np.uint8)
            self.original_img = cv2.imdecode(stream, cv2.IMREAD_COLOR)
            if self.original_img is None: return
            self.reset_points()
            img_disp = self.fit_image_to_box(self.get_display_image(self.original_img))
            self.photo_orig = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(img_disp, cv2.COLOR_BGR2RGB)))
            self.canvas_orig.delete("all")
            self.canvas_orig.create_image(250, 250, image=self.photo_orig, anchor=tk.CENTER)

    def reset_points(self):
        self.points_source = []
        self.canvas_points = []
        if self.photo_orig:
            self.canvas_orig.delete("all")
            self.canvas_orig.create_image(250, 250, image=self.photo_orig, anchor=tk.CENTER)

    def on_canvas_orig_click(self, event):
        if self.original_img is None: return
        h_real, w_real = self.original_img.shape[:2]
        scale = 500 / max(h_real, w_real)
        disp_w, disp_h = int(w_real * scale), int(h_real * scale)
        offset_x, offset_y = (500 - disp_w) // 2, (500 - disp_h) // 2
        if event.x < offset_x or event.x > offset_x + disp_w or event.y < offset_y or event.y > offset_y + disp_h: return
        real_x, real_y = int((event.x - offset_x) / scale), int((event.y - offset_y) / scale)
        if len(self.points_source) < 4:
            self.points_source.append([real_x, real_y])
            self.canvas_points.append((event.x, event.y))
            r = 4
            self.canvas_orig.create_oval(event.x-r, event.y-r, event.x+r, event.y+r, fill="yellow", outline="black")
            if len(self.canvas_points) > 1:
                self.canvas_orig.create_line(self.canvas_points[-2][0], self.canvas_points[-2][1], event.x, event.y, fill="yellow", dash=(4,2))
            if len(self.canvas_points) == 4:
                self.canvas_orig.create_line(event.x, event.y, self.canvas_points[0][0], self.canvas_points[0][1], fill="yellow", dash=(4,2))

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)] 
        diff = np.diff(pts, axis=1)
        rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)] 
        return rect

    def correct_perspective(self):
        if len(self.points_source) != 4: return
        pts = np.array(self.points_source, dtype="float32")
        rect_ordered = self.order_points(pts)
        
        dest_w, dest_h = int(MAP_WIDTH_M * 100), int(MAP_HEIGHT_M * 100)
        points_dest = np.float32([[0, 0], [dest_w - 1, 0], [dest_w - 1, dest_h - 1], [0, dest_h - 1]])
        
        # --- CORRECTION ICI ---
        # On récupère l'image sans distorsion AVANT de calculer la perspective
        src_img = self.get_display_image(self.original_img)
        
        matrix = cv2.getPerspectiveTransform(rect_ordered, points_dest)
        self.warped_img = cv2.warpPerspective(src_img, matrix, (dest_w, dest_h))
        
        # Mise à jour de l'affichage
        img_disp = self.fit_image_to_box(self.warped_img)
        self.photo_orig = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(img_disp, cv2.COLOR_BGR2RGB))) 
        self.canvas_orig.delete("all")
        self.canvas_orig.create_image(250, 250, image=self.photo_orig, anchor=tk.CENTER)

    def process_image(self):
        if self.warped_img is None: return
        gray = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edges = cv2.Canny(blurred, self.canny_low.get(), self.canny_high.get())
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        binary_obstacles = np.zeros_like(gray)
        min_a = self.min_area.get()
        for cnt in contours:
            if cv2.contourArea(cnt) > min_a:
                cv2.drawContours(binary_obstacles, [cnt], -1, 255, thickness=cv2.FILLED)
        
        small_binary = cv2.resize(binary_obstacles, (GRID_W, GRID_H), interpolation=cv2.INTER_NEAREST)
        free_space = cv2.bitwise_not(small_binary) 
        dist = cv2.distanceTransform(free_space, cv2.DIST_L2, 3)
        
        self.costmap_grid = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
        robot_radius_cells = (ROBOT_WIDTH_CM / 2.0) / CELL_SIZE_CM
        safe_margin_cells = SAFE_MARGIN_CM / CELL_SIZE_CM
        max_dist_cells = robot_radius_cells + safe_margin_cells
        
        for y in range(GRID_H):
            for x in range(GRID_W):
                d = dist[y, x]
                if d <= robot_radius_cells:
                    self.costmap_grid[y, x] = 255 
                elif d < max_dist_cells:
                    fraction = (d - robot_radius_cells) / safe_margin_cells
                    self.costmap_grid[y, x] = int(254 * (1.0 - fraction)) 
                else:
                    self.costmap_grid[y, x] = 0 
        
        self.start_pos, self.end_pos, self.current_path = None, None, []
        self.update_costmap_canvas()
        self.lbl_status.config(text="Costmap générée.")

    # === NOUVEAU : ALGORITHME A* INTÉGRÉ ===
    def run_pathfinding(self):
        if self.costmap_grid is None or not self.start_pos or not self.end_pos:
            messagebox.showwarning("Erreur", "Générez la map et placez A et B.")
            return
        
        self.lbl_status.config(text="Calcul cinématique optimisé en cours...")
        self.root.update()

        # --- Configuration Cinématique ---
        L_AXE_M = 0.20    # Tiré de Kinematics.h
        ARC_STEP_M = 0.30 # Longueur de chaque segment de courbe
        MAX_STEER = np.radians(35) # Angle max des servos
        
        # --- Paramètres de discrétisation (pour la performance) ---
        XY_RES = 0.15     # On regroupe les positions par blocs de 15cm
        THETA_RES = np.radians(15) # On regroupe les angles par blocs de 15°

        def to_world(pos):
            return (pos[0] * CELL_SIZE_CM / 100.0, pos[1] * CELL_SIZE_CM / 100.0)

        start_w = to_world(self.start_pos)
        goal_w = to_world(self.end_pos)

        # État initial : (x, y, theta)
        start_state = (start_w[0], start_w[1], 0.0)
        
        # queue = [(priorité, (x, y, theta))]
        queue = [(0, start_state)]
        came_from = {start_state: None}
        cost_so_far = {start_state: 0}
        
        # On utilise un set pour suivre les états déjà "fermés" via la clé discrétisée
        closed_states = set()

        found_goal = None

        while queue:
            _, current = heapq.heappop(queue)
            curr_x, curr_y, curr_theta = current

            # 1. Condition de succès (tolérance de 30cm)
            dist_to_goal = np.sqrt((curr_x - goal_w[0])**2 + (curr_y - goal_w[1])**2)
            if dist_to_goal < 0.35:
                found_goal = current
                break

            # Discrétisation de l'état actuel pour éviter les doublons
            state_key = (round(curr_x / XY_RES), round(curr_y / XY_RES), round(curr_theta / THETA_RES))
            if state_key in closed_states:
                continue
            closed_states.add(state_key)

            # 2. Simulation des commandes (5 directions de braquage)
            for steer in np.linspace(-MAX_STEER, MAX_STEER, 5):
                if abs(steer) < 0.01: # Ligne droite
                    next_x = curr_x + ARC_STEP_M * np.cos(curr_theta)
                    next_y = curr_y + ARC_STEP_M * np.sin(curr_theta)
                    next_theta = curr_theta
                else: # Modèle ICR
                    R = L_AXE_M / np.tan(steer)
                    d_theta = ARC_STEP_M / R
                    next_theta = curr_theta + d_theta
                    next_x = curr_x + R * (np.sin(next_theta) - np.sin(curr_theta))
                    next_y = curr_y - R * (np.cos(next_theta) - np.cos(curr_theta))

                # Vérification des limites et collisions
                gx = int((next_x * 100.0) / CELL_SIZE_CM)
                gy = int((next_y * 100.0) / CELL_SIZE_CM)

                if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                    weight = self.costmap_grid[gy, gx]
                    if weight >= 250: continue # Obstacle détecté

                    next_state = (next_x, next_y, next_theta)
                    
                    # Coût = distance + pénalité de proximité d'obstacle
                    # On ajoute une petite pénalité au braquage pour favoriser les lignes droites
                    steering_penalty = abs(steer) * 0.1
                    new_cost = cost_so_far[current] + ARC_STEP_M + (weight / 50.0) + steering_penalty
                    
                    if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                        cost_so_far[next_state] = new_cost
                        # Heuristique : Distance Euclidienne
                        priority = new_cost + np.sqrt((next_x - goal_w[0])**2 + (next_y - goal_w[1])**2)
                        heapq.heappush(queue, (priority, next_state))
                        came_from[next_state] = current

        # 3. Reconstruction et affichage
        if not found_goal:
            messagebox.showerror("Échec", "Aucun chemin cinématiquement possible.")
            self.lbl_status.config(text="Échec du Pathfinding.")
            return

        path = []
        curr = found_goal
        while curr is not None:
            gx = int((curr[0] * 100.0) / CELL_SIZE_CM)
            gy = int((curr[1] * 100.0) / CELL_SIZE_CM)
            path.append((gx, gy))
            curr = came_from.get(curr)
        
        self.current_path = path[::-1]
        self.update_costmap_canvas()
        self.lbl_status.config(text=f"Succès ! Trajectoire trouvée en {len(self.current_path)} segments.")

    # === UPDATE CANVAS (Modifié pour afficher la trajectoire) ===
    def update_costmap_canvas(self):
        if self.costmap_grid is None: return
        
        disp_grid_large = cv2.resize(self.costmap_grid, (GRID_W*SCALE_UI, GRID_H*SCALE_UI), interpolation=cv2.INTER_NEAREST)
        heatmap_rgb = np.zeros((GRID_H*SCALE_UI, GRID_W*SCALE_UI, 3), dtype=np.uint8)
        heatmap_rgb[:, :, 0] = disp_grid_large       
        heatmap_rgb[:, :, 1] = 0
        heatmap_rgb[:, :, 2] = 255 - disp_grid_large 
        
        alpha = self.overlay_alpha.get() / 100.0
        if alpha < 1.0 and self.warped_img is not None:
            warped_rgb = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2RGB)
            warped_resized = cv2.resize(warped_rgb, (GRID_W*SCALE_UI, GRID_H*SCALE_UI), interpolation=cv2.INTER_AREA)
            final_img = cv2.addWeighted(heatmap_rgb, alpha, warped_resized, 1.0 - alpha, 0)
        else:
            final_img = heatmap_rgb
            
        self.photo_cost = ImageTk.PhotoImage(Image.fromarray(final_img))
        self.canvas_cost.delete("all")
        self.canvas_cost.create_image(0, 0, image=self.photo_cost, anchor=tk.NW)
        
        # NOUVEAU : Dessin de la Trajectoire
        if hasattr(self, 'current_path') and len(self.current_path) > 1:
            points_ui = []
            for p in self.current_path:
                x = p[0] * SCALE_UI + (SCALE_UI//2)
                y = p[1] * SCALE_UI + (SCALE_UI//2)
                points_ui.append((x, y))
            self.canvas_cost.create_line(points_ui, fill="#FFF", width=3, smooth=True)

        # Dessins UI (A et B)
        if self.start_pos:
            cx, cy = self.start_pos[0] * SCALE_UI + (SCALE_UI//2), self.start_pos[1] * SCALE_UI + (SCALE_UI//2)
            self.canvas_cost.create_oval(cx-6, cy-6, cx+6, cy+6, fill="#00FF00", outline="white", width=2)
            self.canvas_cost.create_text(cx, cy-12, text="A", fill="#00FF00", font=("Arial", 10, "bold"))

        if self.end_pos:
            cx, cy = self.end_pos[0] * SCALE_UI + (SCALE_UI//2), self.end_pos[1] * SCALE_UI + (SCALE_UI//2)
            self.canvas_cost.create_oval(cx-6, cy-6, cx+6, cy+6, fill="#FFFFFF", outline="black", width=2)
            self.canvas_cost.create_text(cx, cy-12, text="B", fill="#FFFFFF", font=("Arial", 10, "bold"))

    def on_costmap_mouse(self, event):
        if self.costmap_grid is None: return
        mode = self.right_tool_mode.get()
        if mode == "none": return

        gx, gy = int(event.x // SCALE_UI), int(event.y // SCALE_UI)
        if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H: return

        if mode == "start" and event.type == tk.EventType.ButtonPress: 
            if self.costmap_grid[gy, gx] >= 254:
                messagebox.showwarning("Attention", "Impossible de démarrer dans un mur !")
                return
            self.start_pos = (gx, gy)
            self.current_path = [] # Reset chemin si on bouge A
            self.update_costmap_canvas()
            
        elif mode == "end" and event.type == tk.EventType.ButtonPress: 
            if self.costmap_grid[gy, gx] >= 254:
                messagebox.showwarning("Attention", "Impossible d'arriver dans un mur !")
                return
            self.end_pos = (gx, gy)
            self.current_path = [] # Reset chemin si on bouge B
            self.update_costmap_canvas()
            
        elif mode == "brush": 
            val = self.brush_cost.get() 
            radius = self.brush_size.get() - 1 
            y_min, y_max = max(0, gy - radius), min(GRID_H, gy + radius + 1)
            x_min, x_max = max(0, gx - radius), min(GRID_W, gx + radius + 1)
            self.costmap_grid[y_min:y_max, x_min:x_max] = val
            self.current_path = [] # Reset chemin si on modifie la map
            self.update_costmap_canvas()

# === NOUVEL EXPORT (Avec Point de Départ, Arrivée et Orientation) ===
    def export_code(self):
        if not self.current_path or not self.start_pos or not self.end_pos: 
            messagebox.showerror("Erreur", "Veuillez d'abord calculer une trajectoire avec A et B.")
            return

        # 1. Conversion des points A et B en mètres
        start_x = self.start_pos[0] * (CELL_SIZE_CM / 100.0)
        start_y = self.start_pos[1] * (CELL_SIZE_CM / 100.0)
        
        goal_x = self.end_pos[0] * (CELL_SIZE_CM / 100.0)
        goal_y = self.end_pos[1] * (CELL_SIZE_CM / 100.0)

        # 2. Calcul de l'orientation initiale (Regard pointé vers le cap suivant)
        start_theta = 0.0
        if len(self.current_path) > 1:
            # On prend le 1er et le 2ème point du chemin calculé
            p0 = self.current_path[0]
            p1 = self.current_path[1]
            dx = (p1[0] - p0[0]) * (CELL_SIZE_CM / 100.0)
            dy = (p1[1] - p0[1]) * (CELL_SIZE_CM / 100.0)
            start_theta = np.arctan2(dy, dx)

        # 3. Génération du code C++
        cpp_code = f"// ==========================================\n"
        cpp_code += f"// MISSION VEGA SC317 - TRAJECTOIRE A*\n"
        cpp_code += f"// ==========================================\n\n"
        cpp_code += f"#ifndef MISSION_EXPORT_H\n#define MISSION_EXPORT_H\n\n"
        
        cpp_code += f"// --- Points clés de la mission ---\n"
        cpp_code += f"const float START_X = {start_x:.3f};\n"
        cpp_code += f"const float START_Y = {start_y:.3f};\n"
        cpp_code += f"const float START_THETA = {start_theta:.3f}; // Angle initial (radians)\n\n"
        
        cpp_code += f"const float GOAL_X = {goal_x:.3f};\n"
        cpp_code += f"const float GOAL_Y = {goal_y:.3f};\n\n"
        
        cpp_code += f"// --- Trajectoire ---\n"
        cpp_code += f"struct Waypoint {{\n    float x; // mètres\n    float y; // mètres\n}};\n\n"
        
        cpp_code += f"const int PATH_SIZE = {len(self.current_path)};\n"
        cpp_code += f"const Waypoint MISSION_PATH[PATH_SIZE] = {{\n"
        
        for p in self.current_path:
            real_x = p[0] * (CELL_SIZE_CM / 100.0)
            real_y = p[1] * (CELL_SIZE_CM / 100.0)
            cpp_code += f"    {{{real_x:.3f}f, {real_y:.3f}f}},\n"
            
        cpp_code += "};\n\n#endif\n"
        
        # 4. Sauvegarde
        export_path = os.path.join(os.path.dirname(__file__), "mission_export.h")
        with open(export_path, "w") as f:
            f.write(cpp_code)
            
        messagebox.showinfo("Export Réussi", f"Trajectoire de {len(self.current_path)} points exportée avec coordonnées de départ !")

if __name__ == "__main__":
    root = tk.Tk()
    app = CostmapApp(root)
    root.mainloop()