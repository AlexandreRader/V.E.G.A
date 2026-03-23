import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import os

# --- CONFIGURATION PHYSIQUE DE LA PISTE ---
MAP_WIDTH_M = 4.0   
MAP_HEIGHT_M = 6.0  
CELL_SIZE_CM = 10.0 

# --- CONFIGURATION TRAITEMENT ---
ROBOT_WIDTH_CM = 30.0 # Largeur de votre rover (Zone Mortelle 255)
SAFE_MARGIN_CM = 40.0 # Distance supplémentaire du dégradé (Aura bleue/violette)

# --- CALCULS CONSTANTS ---
GRID_W = int((MAP_WIDTH_M * 100) / CELL_SIZE_CM)  # 40 cases
GRID_H = int((MAP_HEIGHT_M * 100) / CELL_SIZE_CM) # 60 cases
SCALE_UI = 10 

class CostmapApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Station Sol - Éditeur de Mission VEGA SC317")
        self.root.geometry("1300x850") 
        
        self.original_img = None
        self.warped_img = None
        self.costmap_grid = None # Matrice 0 à 255
        
        # Données de mission & couleurs
        self.points_source = [] 
        self.canvas_points = [] 
        self.start_pos = None 
        self.end_pos = None   
        
        self.color_free_bgr = None # Couleur du sol (Pipette)
        self.color_obs_bgr = None  # Couleur de l'obstacle (Pipette)
        
        self.photo_orig = None
        self.photo_cost = None
        
        # Outils UI
        self.left_tool_mode = tk.StringVar(value="perspective")
        self.right_tool_mode = tk.StringVar(value="none")
        self.brush_size = tk.IntVar(value=1)
        
        self.setup_ui()

    def setup_ui(self):
        # 1. BARRE DU HAUT
        btn_frame = tk.Frame(self.root, bg="#f0f0f0")
        btn_frame.pack(side=tk.TOP, fill=tk.X, pady=10)
        btn_style = {"font": ("Arial", 10, "bold"), "fg": "white", "padx": 10, "pady": 5}

        tk.Button(btn_frame, text="1. Charger l'image", command=self.load_image, bg="#4CAF50", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="2. Redresser (Perspective)", command=self.correct_perspective, bg="#9C27B0", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="3. Générer Costmap B/R", command=self.process_image, bg="#2196F3", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="4. Exporter Mission C++", command=self.export_code, bg="#FF9800", **btn_style).pack(side=tk.RIGHT, padx=20)
        
        # 2. ZONE DES CANVASES
        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=20)
        
        # --- GAUCHE : Caméra & Pipettes ---
        orig_frame = tk.Frame(self.canvas_frame)
        orig_frame.pack(side=tk.LEFT, padx=10)
        tk.Label(orig_frame, text="Caméra Originale", font=("Arial", 11, "bold")).pack()
        
        self.canvas_orig = tk.Canvas(orig_frame, width=500, height=500, bg="#ddd", borderwidth=2, relief="groove", cursor="crosshair")
        self.canvas_orig.pack()
        self.canvas_orig.bind("<Button-1>", self.on_canvas_orig_click)

        # Barre d'outils Gauche
        left_toolbar = tk.Frame(orig_frame, pady=5)
        left_toolbar.pack(fill=tk.X)
        tk.Radiobutton(left_toolbar, text="📍 4 Coins", variable=self.left_tool_mode, value="perspective").pack(side=tk.LEFT)
        tk.Button(left_toolbar, text="🔄", command=self.reset_points, font=("Arial", 8)).pack(side=tk.LEFT, padx=5)
        
        tk.Radiobutton(left_toolbar, text="🖌️ Couleur Sol", variable=self.left_tool_mode, value="c_free").pack(side=tk.LEFT, padx=5)
        self.lbl_c_free = tk.Label(left_toolbar, text="██", fg="#d3d3d3", font=("Arial", 12))
        self.lbl_c_free.pack(side=tk.LEFT)
        
        tk.Radiobutton(left_toolbar, text="🖌️ Couleur Obstacle", variable=self.left_tool_mode, value="c_obs").pack(side=tk.LEFT, padx=5)
        self.lbl_c_obs = tk.Label(left_toolbar, text="██", fg="#d3d3d3", font=("Arial", 12))
        self.lbl_c_obs.pack(side=tk.LEFT)

        # --- DROITE : Costmap & Éditeur ---
        cost_frame = tk.Frame(self.canvas_frame)
        cost_frame.pack(side=tk.LEFT, padx=20)
        tk.Label(cost_frame, text=f"Costmap (0=Bleu, 255=Rouge)", font=("Arial", 11, "bold")).pack()
        
        self.canvas_cost = tk.Canvas(cost_frame, width=GRID_W*SCALE_UI, height=GRID_H*SCALE_UI, bg="black", borderwidth=2, relief="groove", cursor="target")
        self.canvas_cost.pack()
        self.canvas_cost.bind("<Button-1>", self.on_costmap_mouse)
        self.canvas_cost.bind("<B1-Motion>", self.on_costmap_mouse)

        # Barre d'outils Droite
        right_toolbar = tk.Frame(cost_frame, pady=5)
        right_toolbar.pack(fill=tk.X)
        
        tk.Radiobutton(right_toolbar, text="Sélection", variable=self.right_tool_mode, value="none").pack(side=tk.LEFT)
        tk.Radiobutton(right_toolbar, text="Mur (+)", variable=self.right_tool_mode, value="add", fg="red").pack(side=tk.LEFT)
        tk.Radiobutton(right_toolbar, text="Sol (-)", variable=self.right_tool_mode, value="erase", fg="blue").pack(side=tk.LEFT)
        tk.Scale(right_toolbar, from_=1, to=5, orient=tk.HORIZONTAL, variable=self.brush_size, length=60, showvalue=0).pack(side=tk.LEFT)
        
        tk.Radiobutton(right_toolbar, text="🟢 A", variable=self.right_tool_mode, value="start", fg="green").pack(side=tk.LEFT, padx=10)
        tk.Radiobutton(right_toolbar, text="🔴 B", variable=self.right_tool_mode, value="end", fg="red").pack(side=tk.LEFT)

        self.lbl_status = tk.Label(self.root, text="Statut : Prêt.", font=("Arial", 9), fg="#777")
        self.lbl_status.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

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
            img_disp = self.fit_image_to_box(self.original_img)
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

        real_x = int((event.x - offset_x) / scale)
        real_y = int((event.y - offset_y) / scale)
        
        mode = self.left_tool_mode.get()

        if mode == "perspective":
            if len(self.points_source) < 4:
                self.points_source.append([real_x, real_y])
                self.canvas_points.append((event.x, event.y))
                r = 4
                self.canvas_orig.create_oval(event.x-r, event.y-r, event.x+r, event.y+r, fill="yellow", outline="black")
                if len(self.canvas_points) > 1:
                    self.canvas_orig.create_line(self.canvas_points[-2][0], self.canvas_points[-2][1], event.x, event.y, fill="yellow", dash=(4,2))
                if len(self.canvas_points) == 4:
                    self.canvas_orig.create_line(event.x, event.y, self.canvas_points[0][0], self.canvas_points[0][1], fill="yellow", dash=(4,2))
                    
        elif mode == "c_free":
            self.color_free_bgr = self.original_img[real_y, real_x]
            b, g, r = self.color_free_bgr
            self.lbl_c_free.config(fg=f"#{r:02x}{g:02x}{b:02x}")
            self.lbl_status.config(text="Couleur du SOL enregistrée.")
            
        elif mode == "c_obs":
            self.color_obs_bgr = self.original_img[real_y, real_x]
            b, g, r = self.color_obs_bgr
            self.lbl_c_obs.config(fg=f"#{r:02x}{g:02x}{b:02x}")
            self.lbl_status.config(text="Couleur de l'OBSTACLE enregistrée.")

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0], rect[2] = pts[np.argmin(s)], pts[np.argmax(s)] 
        diff = np.diff(pts, axis=1)
        rect[1], rect[3] = pts[np.argmin(diff)], pts[np.argmax(diff)] 
        return rect

    def correct_perspective(self):
        if len(self.points_source) != 4: 
            messagebox.showwarning("Erreur", "Placez les 4 points de la piste d'abord.")
            return
            
        pts = np.array(self.points_source, dtype="float32")
        rect_ordered = self.order_points(pts)

        dest_w, dest_h = int(MAP_WIDTH_M * 100), int(MAP_HEIGHT_M * 100)
        points_dest = np.float32([[0, 0], [dest_w - 1, 0], [dest_w - 1, dest_h - 1], [0, dest_h - 1]])
        matrix = cv2.getPerspectiveTransform(rect_ordered, points_dest)
        self.warped_img = cv2.warpPerspective(self.original_img, matrix, (dest_w, dest_h))
        
        img_disp = self.fit_image_to_box(self.warped_img)
        self.photo_orig = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(img_disp, cv2.COLOR_BGR2RGB))) 
        self.canvas_orig.delete("all")
        self.canvas_orig.create_image(250, 250, image=self.photo_orig, anchor=tk.CENTER)
        self.lbl_status.config(text="Image redressée ! Utilisez les pipettes pour sélectionner les couleurs.")

    def process_image(self):
        if self.warped_img is None: 
            messagebox.showerror("Erreur", "Redressez l'image d'abord.")
            return
            
        if self.color_free_bgr is None or self.color_obs_bgr is None:
            messagebox.showerror("Erreur", "Utilisez les pipettes pour sélectionner la couleur du sol ET de l'obstacle.")
            return
            
        # 1. Conversion de l'image et des couleurs en LAB (Luminance, A, B)
        # C'est beaucoup plus robuste aux ombres et aux reflets que le RGB !
        warped_lab = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2Lab).astype(np.float32)
        c_free_lab = cv2.cvtColor(np.uint8([[self.color_free_bgr]]), cv2.COLOR_BGR2Lab)[0][0].astype(np.float32)
        c_obs_lab = cv2.cvtColor(np.uint8([[self.color_obs_bgr]]), cv2.COLOR_BGR2Lab)[0][0].astype(np.float32)

        # 2. Calcul de la distance colorimétrique de chaque pixel
        dist_free = np.linalg.norm(warped_lab - c_free_lab, axis=2)
        dist_obs = np.linalg.norm(warped_lab - c_obs_lab, axis=2)

        # 3. Création du masque Binaire : Si c'est plus proche de l'obstacle, c'est 255
        binary = np.where(dist_obs < dist_free, 255, 0).astype(np.uint8)
        
        # 4. Redimensionnement à la grille ESP32
        small_binary = cv2.resize(binary, (GRID_W, GRID_H), interpolation=cv2.INTER_NEAREST)
        
        # 5. Distance Transform (Gradient d'inflation)
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
        
        self.start_pos = None
        self.end_pos = None
        self.update_costmap_canvas()
        self.lbl_status.configure(text="Costmap Bleu/Rouge générée par détection de couleurs !")

    def update_costmap_canvas(self):
        if self.costmap_grid is None: return
        
        # --- CRÉATION DU DÉGRADÉ BLEU -> ROUGE PUR ---
        # Agrandissement pour l'interface
        disp_grid_large = cv2.resize(self.costmap_grid, (GRID_W*SCALE_UI, GRID_H*SCALE_UI), interpolation=cv2.INTER_NEAREST)
        
        # Image RGB vide
        heatmap_rgb = np.zeros((GRID_H*SCALE_UI, GRID_W*SCALE_UI, 3), dtype=np.uint8)
        # Canal ROUGE : augmente avec le coût
        heatmap_rgb[:, :, 0] = disp_grid_large
        # Canal VERT : toujours 0 (pour éviter le jaune/cyan)
        heatmap_rgb[:, :, 1] = 0
        # Canal BLEU : inverse du coût (max quand c'est libre)
        heatmap_rgb[:, :, 2] = 255 - disp_grid_large
        
        img_pil = Image.fromarray(heatmap_rgb)
        self.photo_cost = ImageTk.PhotoImage(img_pil)
        self.canvas_cost.delete("all")
        self.canvas_cost.create_image(0, 0, image=self.photo_cost, anchor=tk.NW)
        
        # Dessiner le point de DEPART (Vert fluo)
        if self.start_pos:
            cx, cy = self.start_pos[0] * SCALE_UI + (SCALE_UI//2), self.start_pos[1] * SCALE_UI + (SCALE_UI//2)
            self.canvas_cost.create_oval(cx-6, cy-6, cx+6, cy+6, fill="#00FF00", outline="white", width=2)
            self.canvas_cost.create_text(cx, cy-12, text="A", fill="#00FF00", font=("Arial", 10, "bold"))

        # Dessiner le point d'ARRIVEE (Blanc pour contraster)
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
            if self.costmap_grid[gy, gx] == 255:
                messagebox.showwarning("Attention", "Impossible de démarrer dans un mur (Rouge 255) !")
                return
            self.start_pos = (gx, gy)
            self.update_costmap_canvas()
            
        elif mode == "end" and event.type == tk.EventType.ButtonPress: 
            if self.costmap_grid[gy, gx] == 255:
                messagebox.showwarning("Attention", "Impossible d'arriver dans un mur (Rouge 255) !")
                return
            self.end_pos = (gx, gy)
            self.update_costmap_canvas()
            
        elif mode in ["add", "erase"]: 
            val = 255 if mode == "add" else 0
            radius = self.brush_size.get() - 1 
            y_min, y_max = max(0, gy - radius), min(GRID_H, gy + radius + 1)
            x_min, x_max = max(0, gx - radius), min(GRID_W, gx + radius + 1)
            self.costmap_grid[y_min:y_max, x_min:x_max] = val
            self.update_costmap_canvas()

    def export_code(self):
        if self.costmap_grid is None: 
            messagebox.showerror("Erreur", "Générez la costmap d'abord.")
            return
            
        if not self.start_pos or not self.end_pos:
            if not messagebox.askyesno("Attention", "Départ ou Arrivée non défini ! Exporter quand même ?"): return

        sx, sy = self.start_pos if self.start_pos else (-1, -1)
        ex, ey = self.end_pos if self.end_pos else (-1, -1)

        cpp_code = f"// ==========================================\n"
        cpp_code += f"// MISSION VEGA SC317 - COSTMAP 0-255\n"
        cpp_code += f"// Dimensions réelles: {MAP_WIDTH_M}m de large x {MAP_HEIGHT_M}m de long\n"
        cpp_code += f"// 1 case = {CELL_SIZE_CM}x{CELL_SIZE_CM} cm\n"
        cpp_code += f"// ==========================================\n\n"
        
        cpp_code += f"#define MAP_WIDTH {GRID_W}\n"
        cpp_code += f"#define MAP_HEIGHT {GRID_H}\n\n"
        
        cpp_code += f"#define START_X {sx}\n"
        cpp_code += f"#define START_Y {sy}\n"
        cpp_code += f"#define GOAL_X {ex}\n"
        cpp_code += f"#define GOAL_Y {ey}\n\n"

        cpp_code += f"uint8_t costmap[MAP_HEIGHT][MAP_WIDTH] = {{\n"
        for row in self.costmap_grid:
            row_str = ", ".join(f"{val:3}" for val in row)
            cpp_code += f"    {{{row_str}}},\n"
        cpp_code += "};\n"
        
        export_path = os.path.join(os.path.dirname(__file__), "mission_export.h")
        with open(export_path, "w") as f:
            f.write(cpp_code)
            
        messagebox.showinfo("Export Réussi", f"Fichier sauvegardé !\n{export_path}")

if __name__ == "__main__":
    root = tk.Tk()
    app = CostmapApp(root)
    root.mainloop()