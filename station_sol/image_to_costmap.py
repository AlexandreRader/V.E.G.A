import queue
import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import os
import glob
import heapq

# --- CONFIGURATION PHYSIQUE DE LA PISTE ---
MAP_WIDTH_M = 4.0   
MAP_HEIGHT_M = 4.0  
CELL_SIZE_CM = 5.0 

# --- CONFIGURATION CALIBRATION ---
CHESSBOARD_SIZE = (7, 7)    
SQUARE_SIZE = 1.0           

# --- CONFIGURATION TRAITEMENT ---
ROBOT_WIDTH_CM = 30.0 
SAFE_MARGIN_CM = 20.0 

# --- CALCULS CONSTANTS ---
GRID_W = int((MAP_WIDTH_M * 100) / CELL_SIZE_CM)  
GRID_H = int((MAP_HEIGHT_M * 100) / CELL_SIZE_CM) 
SCALE_UI = 10 


class ZoomableCanvas:
    """Classe optimisée pour ajouter le support du Zoom, Pan et clics filtrés"""
    def __init__(self, canvas, on_click_callback, allow_drag_click=True):
        self.canvas = canvas
        self.on_click_callback = on_click_callback
        self.allow_drag_click = allow_drag_click # Permet de bloquer le drag pour éviter les multi-clics
        
        self.image_id = None
        self.cv_img = None
        self.tk_img = None
        
        self.zoom_level = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.start_x = 0
        self.start_y = 0
        
        self._redraw_pending = False
        
        # Bind des commandes de déplacement et zoom (Clic Droit et Molette)
        self.canvas.bind("<ButtonPress-3>", self.start_pan)
        self.canvas.bind("<B3-Motion>", self.execute_pan)
        self.canvas.bind("<MouseWheel>", self.zoom)
        self.canvas.bind("<Button-4>", self.zoom)   
        self.canvas.bind("<Button-5>", self.zoom)   
        
        # Clic gauche unitaire
        self.canvas.bind("<ButtonPress-1>", self.handle_click_press)
        
        # Le mouvement au clic gauche n'est activé que si explicitement autorisé (ex: Pinceau Costmap)
        if self.allow_drag_click:
            self.canvas.bind("<B1-Motion>", self.handle_click_drag)

    def set_image(self, cv_img, reset_view=False):
        self.cv_img = cv_img
        if reset_view:
            self.zoom_level = 1.0
            self.pan_x = 0
            self.pan_y = 0
        self.request_redraw()

    def request_redraw(self):
        if not self._redraw_pending:
            self._redraw_pending = True
            self.canvas.after(10, self._execute_redraw)

    def _execute_redraw(self):
        self._redraw_pending = False
        if self.cv_img is None:
            return
            
        h, w = self.cv_img.shape[:2]
        new_w = int(w * self.zoom_level)
        new_h = int(h * self.zoom_level)
        
        if new_w <= 0 or new_h <= 0:
            return
            
        if self.tk_img is not None:
            del self.tk_img
            self.tk_img = None
            
        try:
            resized = cv2.resize(self.cv_img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            rgb_img = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            
            pil_img = Image.fromarray(rgb_img)
            self.tk_img = ImageTk.PhotoImage(pil_img)
            
            self.canvas.delete("all")
            self.image_id = self.canvas.create_image(self.pan_x, self.pan_y, image=self.tk_img, anchor=tk.NW)
        except Exception as e:
            print(f"⚠️ Erreur graphique lors du rendu : {e}")

    def start_pan(self, event):
        self.start_x = event.x
        self.start_y = event.y

    def execute_pan(self, event):
        dx = event.x - self.start_x
        dy = event.y - self.start_y
        self.pan_x += dx
        self.pan_y += dy
        self.start_x = event.x
        self.start_y = event.y
        self.request_redraw()

    def zoom(self, event):
        if event.num == 4 or event.delta > 0:
            factor = 1.15
        elif event.num == 5 or event.delta < 0:
            factor = 0.85
        else:
            return
            
        if self.zoom_level * factor < 0.2 or self.zoom_level * factor > 20.0:
            return
            
        mouse_x = event.x
        mouse_y = event.y
        
        self.pan_x = int(mouse_x - (mouse_x - self.pan_x) * factor)
        self.pan_y = int(mouse_y - (mouse_y - self.pan_y) * factor)
        self.zoom_level *= factor
        
        self.request_redraw()

    def handle_click_press(self, event):
        self.process_click(event, event_type=tk.EventType.ButtonPress)

    def handle_click_drag(self, event):
        self.process_click(event, event_type=tk.EventType.Motion)

    def process_click(self, event, event_type):
        if self.cv_img is None:
            return
        real_x = int((event.x - self.pan_x) / self.zoom_level)
        real_y = int((event.y - self.pan_y) / self.zoom_level)
        
        h, w = self.cv_img.shape[:2]
        if 0 <= real_x < w and 0 <= real_y < h:
            virtual_event = tk.Event()
            virtual_event.x = real_x
            virtual_event.y = real_y
            virtual_event.type = event_type
            self.on_click_callback(virtual_event)


class CostmapApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Station Sol - Éditeur de Mission VEGA SC317 (Lissé & Couleurs Corrigées)")
        self.root.geometry("1400x850") 

        self.start_angle = tk.IntVar(value=0) 
        self.end_angle = tk.IntVar(value=0)   
        
        self.original_img = None
        self.warped_img = None
        self.costmap_grid = None 
        self.current_path = [] 
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_file = None
        
        self.points_source = [] 
        self.start_pos = None 
        self.end_pos = None   
        
        # Outils
        self.right_tool_mode = tk.StringVar(value="none")
        self.brush_size = tk.IntVar(value=3)
        self.brush_cost = tk.IntVar(value=255) 
        self.overlay_alpha = tk.IntVar(value=70) 
        
        # Canny
        self.canny_low = tk.IntVar(value=50)
        self.canny_high = tk.IntVar(value=150)
        self.min_area = tk.IntVar(value=500) 
        
        self.setup_ui()

    def setup_ui(self):
        btn_frame = tk.Frame(self.root, bg="#f0f0f0")
        btn_frame.pack(side=tk.TOP, fill=tk.X, pady=10)
        btn_style = {"font": ("Arial", 10, "bold"), "fg": "white", "padx": 10, "pady": 5}

        tk.Button(btn_frame, text="1. Charger l'image", command=self.load_image, bg="#4CAF50", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="Calibrer Caméra", command=self.calibrate_camera, bg="#607D8B", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="Charger Calib.", command=self.load_calibration, bg="#795548", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="2. Redresser (Perspective)", command=self.correct_perspective, bg="#9C27B0", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="3. Générer Costmap", command=self.process_image, bg="#2196F3", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="4. Calculer Trajectoire", command=self.run_pathfinding, bg="#e74c3c", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="5. Exporter Mission C++", command=self.export_code, bg="#FF9800", **btn_style).pack(side=tk.RIGHT, padx=20)
        
        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=20)
        
        # GAUCHE : Bloc d'origine (SÉCURISÉ : allow_drag_click=False pour éviter les multi-clics fantômes)
        orig_frame = tk.Frame(self.canvas_frame)
        orig_frame.pack(side=tk.LEFT, padx=10)
        tk.Label(orig_frame, text="Caméra Originale (Clic gauche franc pour les 4 coins)", font=("Arial", 11, "bold")).pack()
        
        self.canvas_orig = tk.Canvas(orig_frame, width=500, height=500, bg="#ddd", borderwidth=2, relief="groove")
        self.canvas_orig.pack()
        self.zoom_orig = ZoomableCanvas(self.canvas_orig, self.on_canvas_orig_click, allow_drag_click=False)

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

        # DROITE : Costmap (allow_drag_click=True pour permettre de peindre au pinceau en glissant)
        cost_frame = tk.Frame(self.canvas_frame)
        cost_frame.pack(side=tk.LEFT, padx=20)
        tk.Label(cost_frame, text="Costmap Intégrée", font=("Arial", 11, "bold")).pack()
        
        self.canvas_cost = tk.Canvas(cost_frame, width=GRID_W*SCALE_UI, height=GRID_H*SCALE_UI, bg="black", borderwidth=2, relief="groove")
        self.canvas_cost.pack()
        self.zoom_cost = ZoomableCanvas(self.canvas_cost, self.on_costmap_mouse, allow_drag_click=True)

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
        tk.Radiobutton(bottom_right, text="🟢 A (Départ)", variable=self.right_tool_mode, value="start", fg="green", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(bottom_right, text="🔴 B (Cible)", variable=self.right_tool_mode, value="end", fg="red", font=("Arial", 9, "bold")).pack(side=tk.LEFT)
        tk.Label(bottom_right, text="  |  Transparence :").pack(side=tk.LEFT, padx=(10, 0))
        tk.Scale(bottom_right, from_=0, to=100, orient=tk.HORIZONTAL, variable=self.overlay_alpha, length=120, showvalue=0, command=lambda v: self.update_costmap_canvas()).pack(side=tk.LEFT)

        self.lbl_status = tk.Label(self.root, text="Statut : Prêt.", font=("Arial", 9), fg="#777")
        self.lbl_status.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        tk.Label(bottom_right, text=" | Cap A (°):").pack(side=tk.LEFT, padx=(5,0))
        tk.Scale(bottom_right, from_=-180, to=180, orient=tk.HORIZONTAL, variable=self.start_angle, length=90, showvalue=1, command=lambda v: self.update_costmap_canvas()).pack(side=tk.LEFT)
        
        tk.Label(bottom_right, text=" Cap B (°):").pack(side=tk.LEFT, padx=(5,0))
        tk.Scale(bottom_right, from_=-180, to=180, orient=tk.HORIZONTAL, variable=self.end_angle, length=90, showvalue=1, command=lambda v: self.update_costmap_canvas()).pack(side=tk.LEFT)

    def get_display_image(self, img):
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            return cv2.undistort(img, self.camera_matrix, self.dist_coeffs)
        return img

    def load_image(self):
        path = filedialog.askopenfilename(filetypes=[("Images", "*.jpg *.jpeg *.png")])
        if path:
            stream = np.fromfile(path, dtype=np.uint8)
            self.original_img = cv2.imdecode(stream, cv2.IMREAD_COLOR)
            if self.original_img is None: return
            self.reset_points()
            self.zoom_orig.set_image(self.get_display_image(self.original_img), reset_view=True)

    def redraw_original_image(self):
        if self.original_img is None: return
        self.zoom_orig.set_image(self.get_display_image(self.original_img))

    def calibrate_camera(self):
        image_paths = list(filedialog.askopenfilenames(title="Sélectionner images de calibration", filetypes=[("Images", "*.jpg *.jpeg *.png *.JPG *.JPEG *.PNG")]))
        folder = None
        if not image_paths:
            folder = filedialog.askdirectory(title="Choisir dossier images de calibration")
            if not folder: return
            image_paths = sorted(glob.glob(os.path.join(folder, "*.jpg")) + glob.glob(os.path.join(folder, "*.png")))
        elif image_paths:
            folder = os.path.dirname(image_paths[0])

        if len(image_paths) < 3:
            messagebox.showerror("Erreur", f"Au moins 3 images nécessaires. Trouvées : {len(image_paths)}")
            return

        pattern = CHESSBOARD_SIZE
        objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * SQUARE_SIZE

        objpoints, imgpoints = [], []
        valid_images = 0

        for path in image_paths:
            stream = np.fromfile(path, dtype=np.uint8)
            img = cv2.imdecode(stream, cv2.IMREAD_COLOR)
            if img is None: continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                valid_images += 1
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                objpoints.append(objp)
                imgpoints.append(corners2)

        if valid_images < 3:
            messagebox.showerror("Erreur calibration", "Aucun damier détecté.")
            return

        ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        self.camera_matrix = mtx
        self.dist_coeffs = dist
        self.calibration_file = os.path.join(folder, "camera_calibration.npz")
        np.savez(self.calibration_file, camera_matrix=mtx, dist_coeffs=dist)
        messagebox.showinfo("Succès", f"Calibration enregistrée (RMS: {ret:.4f})")
        if self.original_img is not None: self.redraw_original_image()

    def load_calibration(self):
        path = filedialog.askopenfilename(filetypes=[("NumPy file", "*.npz")], title="Charger calibration camera")
        if not path: return
        try:
            data = np.load(path)
            self.camera_matrix = data["camera_matrix"]
            self.dist_coeffs = data["dist_coeffs"]
            self.calibration_file = path
            if self.original_img is not None: self.redraw_original_image()
        except Exception as e:
            messagebox.showerror("Erreur", f"Erreur de chargement :\n{e}")

    def reset_points(self):
        self.points_source = []
        if self.original_img is not None:
            self.zoom_orig.set_image(self.get_display_image(self.original_img))

    def on_canvas_orig_click(self, event):
        # Sécurité anti multi-clics : on n'enregistre que sur un appui franc (ButtonPress)
        if event.type != tk.EventType.ButtonPress:
            return
            
        if len(self.points_source) < 4:
            self.points_source.append([event.x, event.y])
            working_img = self.get_display_image(self.original_img).copy()
            for i, p in enumerate(self.points_source):
                cv2.circle(working_img, (p[0], p[1]), 6, (0, 0, 255), -1)
                cv2.putText(working_img, str(i+1), (p[0]+10, p[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                if i > 0:
                    cv2.line(working_img, (self.points_source[i-1][0], self.points_source[i-1][1]), (p[0], p[1]), (0, 255, 255), 2)
            if len(self.points_source) == 4:
                cv2.line(working_img, (self.points_source[3][0], self.points_source[3][1]), (self.points_source[0][0], self.points_source[0][1]), (0, 255, 255), 2)
            self.zoom_orig.set_image(working_img)

    def correct_perspective(self):
        if len(self.points_source) != 4:
            messagebox.showwarning("Erreur", "Cliquez sur les 4 coins.")
            return
        rect_ordered = np.array(self.points_source, dtype="float32")
        dest_w, dest_h = int(MAP_WIDTH_M * 100), int(MAP_HEIGHT_M * 100)
        points_dest = np.float32([
            [0, dest_h - 1],
            [0, 0],
            [dest_w - 1, 0],
            [dest_w - 1, dest_h - 1]
        ])
        src_img = self.get_display_image(self.original_img)
        matrix = cv2.getPerspectiveTransform(rect_ordered, points_dest)
        self.warped_img = cv2.warpPerspective(src_img, matrix, (dest_w, dest_h))
        self.zoom_orig.set_image(self.warped_img, reset_view=True)

    def process_image(self):
        if getattr(self, 'warped_img', None) is None: 
            messagebox.showwarning("Attention", "Redressez d'abord la perspective !")
            return
        gray = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edges = cv2.Canny(blurred, self.canny_low.get(), self.canny_high.get())
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contour_results = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_results[0] if len(contour_results) == 2 else contour_results[1]
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
        self.update_costmap_canvas(reset_zoom_view=True)
        self.lbl_status.config(text="Costmap générée. Sélectionnez l'outil 🟢 A ou 🔴 B pour placer vos points.")

    def run_pathfinding(self):
        if self.costmap_grid is None or not self.start_pos or not self.end_pos:
            messagebox.showwarning("Erreur", "Générez la map et placez A et B.")
            return
        self.lbl_status.config(text="Calcul de la trajectoire...")
        self.root.update()
        L_AXE_M = 0.20    
        ARC_STEP_M = 0.30 
        MAX_STEER = np.radians(35) 
        XY_RES = 0.15     
        THETA_RES = np.radians(15) 

        def to_world(pos):
            return (pos[0] * CELL_SIZE_CM / 100.0, (GRID_H - 1 - pos[1]) * CELL_SIZE_CM / 100.0)

        start_w = to_world(self.start_pos)
        goal_w = to_world(self.end_pos)

        start_state = (start_w[0], start_w[1], np.radians(self.start_angle.get()))
        queue = [(0, start_state)]
        came_from = {start_state: None}
        cost_so_far = {start_state: 0}
        closed_states = set()
        found_goal = None

        while queue:
            _, current = heapq.heappop(queue)
            curr_x, curr_y, curr_theta = current
            dist_to_goal = np.sqrt((curr_x - goal_w[0])**2 + (curr_y - goal_w[1])**2)
            
            # 🎯 RECALIBRAGE : On passe le seuil de 35 cm à 8 cm (0.08)
            # L'algorithme A* va chercher des points jusqu'à toucher virtuellement la cible B
            if dist_to_goal < 0.08:
                found_goal = current
                break
            state_key = (round(curr_x / XY_RES), round(curr_y / XY_RES), round(curr_theta / THETA_RES))
            if state_key in closed_states: continue
            closed_states.add(state_key)
            for steer in np.linspace(-MAX_STEER, MAX_STEER, 5):
                if abs(steer) < 0.01:
                    next_x = curr_x + ARC_STEP_M * np.cos(curr_theta)
                    next_y = curr_y + ARC_STEP_M * np.sin(curr_theta)
                    next_theta = curr_theta
                else:
                    R = L_AXE_M / np.tan(steer)
                    d_theta = ARC_STEP_M / R
                    next_theta = curr_theta + d_theta
                    next_x = curr_x + R * (np.sin(next_theta) - np.sin(curr_theta))
                    next_y = curr_y - R * (np.cos(next_theta) - np.cos(curr_theta))
                
                gx = int((next_x * 100.0) / CELL_SIZE_CM)
                gy = int(GRID_H - 1 - (next_y * 100.0) / CELL_SIZE_CM)
                if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                    weight = self.costmap_grid[gy, gx]
                    if weight >= 250: continue 
                    next_state = (next_x, next_y, next_theta)
                    steering_penalty = abs(steer) * 0.1
                    # ANCIEN CODE :
                    # new_cost = cost_so_far[current] + ARC_STEP_M + (weight / 50.0) + steering_penalty

                    # NOUVEAU CODE (Pénalité exponentielle) :
                    # On transforme weight (0-255) en un coût exponentiel
                    cost_factor = (weight / 255.0) ** 3.0  # Plus le poids est proche de 255, plus le coût explose
                    new_cost = cost_so_far[current] + (ARC_STEP_M * (1.0 + cost_factor * 10.0)) + steering_penalty
                    if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                        cost_so_far[next_state] = new_cost
                        priority = new_cost + np.sqrt((next_x - goal_w[0])**2 + (next_y - goal_w[1])**2)
                        heapq.heappush(queue, (priority, next_state))
                        came_from[next_state] = current

        if not found_goal:
            messagebox.showerror("Échec", "Aucun chemin cinématiquement possible.")
            return

        path = []
        curr = found_goal
        while curr is not None:
            gx = int(round((curr[0] * 100.0) / CELL_SIZE_CM))
            gy = int(round(GRID_H - 1 - (curr[1] * 100.0) / CELL_SIZE_CM))
            path.append((gx, gy))
            curr = came_from.get(curr)
        self.current_path = path[::-1]
        
        total_dist = 0.0
        for i in range(1, len(self.current_path)):
            total_dist += np.sqrt((self.current_path[i][0] - self.current_path[i-1][0])**2 + (self.current_path[i][1] - self.current_path[i-1][1])**2) * (CELL_SIZE_CM / 100.0)
            
        self.update_costmap_canvas()
        self.lbl_status.config(text=f"Trajectoire trouvée ! Distance : {total_dist:.2f} m | Points : {len(self.current_path)}")

    def update_costmap_canvas(self, reset_zoom_view=False):
        if self.costmap_grid is None: return
        
        disp_grid_large = cv2.resize(self.costmap_grid, (GRID_W*SCALE_UI, GRID_H*SCALE_UI), interpolation=cv2.INTER_NEAREST)
        heatmap_rgb = np.zeros((GRID_H*SCALE_UI, GRID_W*SCALE_UI, 3), dtype=np.uint8)
        
        # 🎯 CORRECTIF BGR -> RGB POUR TINTER : Les obstacles (255) deviennent ROUGES, l'espace libre (0) devient BLEU
        heatmap_rgb[:, :, 0] = disp_grid_large       # Canal R (Rouge) = Obstacles hauts
        heatmap_rgb[:, :, 2] = 255 - disp_grid_large # Canal B (Bleu) = Espace libre
        
        alpha = self.overlay_alpha.get() / 100.0
        if alpha < 1.0 and self.warped_img is not None:
            warped_rgb = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2RGB)
            warped_resized = cv2.resize(warped_rgb, (GRID_W*SCALE_UI, GRID_H*SCALE_UI), interpolation=cv2.INTER_AREA)
            final_img = cv2.addWeighted(heatmap_rgb, alpha, warped_resized, 1.0 - alpha, 0)
        else:
            final_img = heatmap_rgb
            
        if hasattr(self, 'current_path') and len(self.current_path) > 1:
            for i in range(1, len(self.current_path)):
                p1 = (self.current_path[i-1][0]*SCALE_UI + SCALE_UI//2, self.current_path[i-1][1]*SCALE_UI + SCALE_UI//2)
                p2 = (self.current_path[i][0]*SCALE_UI + SCALE_UI//2, self.current_path[i][1]*SCALE_UI + SCALE_UI//2)
                cv2.line(final_img, p1, p2, (255, 255, 255), 2)

        arrow_length = 30
        if self.start_pos:
            cx = self.start_pos[0] * SCALE_UI + (SCALE_UI//2)
            cy = self.start_pos[1] * SCALE_UI + (SCALE_UI//2)
            theta_start = np.radians(self.start_angle.get())
            ex = int(cx + arrow_length * np.cos(theta_start))
            ey = int(cy - arrow_length * np.sin(theta_start))
            cv2.arrowedLine(final_img, (cx, cy), (ex, ey), (0, 255, 0), 3, tipLength=0.3)
            cv2.circle(final_img, (cx, cy), 6, (0, 255, 0), -1)
            cv2.putText(final_img, "A", (cx-5, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if self.end_pos:
            cx = self.end_pos[0] * SCALE_UI + (SCALE_UI//2)
            cy = self.end_pos[1] * SCALE_UI + (SCALE_UI//2)
            theta_end = np.radians(self.end_angle.get())
            ex = int(cx + arrow_length * np.cos(theta_end))
            ey = int(cy - arrow_length * np.sin(theta_end))
            cv2.arrowedLine(final_img, (cx, cy), (ex, ey), (255, 255, 255), 3, tipLength=0.3)
            cv2.circle(final_img, (cx, cy), 6, (255, 255, 255), -1)
            cv2.putText(final_img, "B", (cx-5, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # OpenCV travaille en BGR en interne, mais l'affichage final s'attend à du RGB. 
        # Pour que la conversion finale de ZoomableCanvas fonctionne, on repasse en BGR.
        final_bgr = cv2.cvtColor(final_img, cv2.COLOR_RGB2BGR)
        self.zoom_cost.set_image(final_bgr, reset_view=reset_zoom_view)

    def on_costmap_mouse(self, event):
        if self.costmap_grid is None: return
        mode = self.right_tool_mode.get()
        if mode == "none": return

        # Sécurisation : pour placer A ou B, on exige un clic franc, pas un glissement
        if mode in ["start", "end"] and event.type != tk.EventType.ButtonPress:
            return

        gx, gy = int(event.x // SCALE_UI), int(event.y // SCALE_UI)
        if gx < 0 or gx >= GRID_W or gy < 0 or gy >= GRID_H: return

        if mode == "start" and event.type == tk.EventType.ButtonPress: 
            self.start_pos = (gx, gy)
            self.current_path = []
            self.update_costmap_canvas()
            self.lbl_status.config(text=f"🟢 Point de départ A posé en case [{gx}, {GRID_H - 1 - gy}]")
        elif mode == "end" and event.type == tk.EventType.ButtonPress: 
            self.end_pos = (gx, gy)
            self.current_path = []
            self.update_costmap_canvas()
            self.lbl_status.config(text=f"🔴 Point cible B posé en case [{gx}, {GRID_H - 1 - gy}]")
        elif mode == "brush": 
            val = self.brush_cost.get() 
            radius = self.brush_size.get() - 1 
            y_min, y_max = max(0, gy - radius), min(GRID_H, gy + radius + 1)
            x_min, x_max = max(0, gx - radius), min(GRID_W, gx + radius + 1)
            self.costmap_grid[y_min:y_max, x_min:x_max] = val
            self.current_path = []
            self.update_costmap_canvas()

    def export_code(self):
        if not self.current_path or not self.start_pos or not self.end_pos: 
            messagebox.showerror("Erreur", "Veuillez d'abord calculer une trajectoire.")
            return
        scale_factor = CELL_SIZE_CM / 100.0
        start_x = self.start_pos[0] * scale_factor
        start_y = (GRID_H - 1 - self.start_pos[1]) * scale_factor
        start_theta = np.radians(self.start_angle.get())
        goal_x = self.end_pos[0] * scale_factor
        goal_y = (GRID_H - 1 - self.end_pos[1]) * scale_factor
        goal_theta = np.radians(self.end_angle.get())
        
        mission_str = f"M{start_x:.2f},{start_y:.2f},{start_theta:.2f},{goal_x:.2f},{goal_y:.2f},{goal_theta:.2f};"
        for p in self.current_path:
            real_x = p[0] * scale_factor
            real_y = (GRID_H - 1 - p[1]) * scale_factor
            mission_str += f"{real_x:.2f},{real_y:.2f};"
        self.show_export_dialog(mission_str)

    def show_export_dialog(self, mission_str):
        top = tk.Toplevel(self.root)
        top.title("🚀 Mission Prête !")
        top.geometry("600x200")
        tk.Label(top, text=f"Longueur : {len(mission_str)} caractères.", font=("Arial", 10, "bold")).pack(pady=10)
        text_area = tk.Text(top, height=5, wrap=tk.WORD, font=("Consolas", 10))
        text_area.pack(padx=20, pady=5, fill=tk.BOTH, expand=True)
        text_area.insert(tk.END, mission_str)
        text_area.tag_add("sel", "1.0", "end")
        text_area.focus_set()
        tk.Button(top, text="Fermer", command=top.destroy).pack(pady=10)


if __name__ == "__main__":
    root = tk.Tk()
    app = CostmapApp(root)
    root.mainloop()