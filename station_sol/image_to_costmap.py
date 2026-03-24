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
ROBOT_WIDTH_CM = 30.0 
SAFE_MARGIN_CM = 40.0 

# --- CALCULS CONSTANTS ---
GRID_W = int((MAP_WIDTH_M * 100) / CELL_SIZE_CM)  
GRID_H = int((MAP_HEIGHT_M * 100) / CELL_SIZE_CM) 
SCALE_UI = 10 

class CostmapApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Station Sol - Éditeur de Mission VEGA SC317 (Détection de Contours + Overlay)")
        self.root.geometry("1350x850") 
        
        self.original_img = None
        self.warped_img = None
        self.costmap_grid = None 
        
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
        
        # NOUVEAU : Transparence de l'overlay (100 = Costmap seule, 0 = Image réelle seule)
        self.overlay_alpha = tk.IntVar(value=70) 
        
        # Variables Canny & Contours
        self.canny_low = tk.IntVar(value=50)
        self.canny_high = tk.IntVar(value=150)
        self.min_area = tk.IntVar(value=500) 
        
        self.setup_ui()

    def setup_ui(self):
        btn_frame = tk.Frame(self.root, bg="#f0f0f0")
        btn_frame.pack(side=tk.TOP, fill=tk.X, pady=10)
        btn_style = {"font": ("Arial", 10, "bold"), "fg": "white", "padx": 10, "pady": 5}

        tk.Button(btn_frame, text="1. Charger l'image", command=self.load_image, bg="#4CAF50", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="2. Redresser (Perspective)", command=self.correct_perspective, bg="#9C27B0", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="3. Générer Costmap (Contours)", command=self.process_image, bg="#2196F3", **btn_style).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="4. Exporter Mission C++", command=self.export_code, bg="#FF9800", **btn_style).pack(side=tk.RIGHT, padx=20)
        
        self.canvas_frame = tk.Frame(self.root)
        self.canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=20)
        
        # --- GAUCHE : Caméra & Outils Canny ---
        orig_frame = tk.Frame(self.canvas_frame)
        orig_frame.pack(side=tk.LEFT, padx=10)
        tk.Label(orig_frame, text="Caméra Originale (Cliquez les 4 coins)", font=("Arial", 11, "bold")).pack()
        
        self.canvas_orig = tk.Canvas(orig_frame, width=500, height=500, bg="#ddd", borderwidth=2, relief="groove", cursor="crosshair")
        self.canvas_orig.pack()
        self.canvas_orig.bind("<Button-1>", self.on_canvas_orig_click)

        # Barre de réglages de la Détection de Contours
        left_toolbar = tk.Frame(orig_frame, pady=5)
        left_toolbar.pack(fill=tk.X)
        tk.Button(left_toolbar, text="🔄 Effacer Cadre", command=self.reset_points, font=("Arial", 9)).pack(side=tk.LEFT, padx=5)
        
        param_frame = tk.Frame(orig_frame, pady=5, bd=1, relief="ridge")
        param_frame.pack(fill=tk.X, pady=5)
        tk.Label(param_frame, text="⚙️ Réglages Détection (Canny) :", font=("Arial", 9, "bold")).pack(anchor="w", padx=5)
        
        row1 = tk.Frame(param_frame)
        row1.pack(fill=tk.X, padx=5)
        tk.Label(row1, text="Sensibilité (Bas):", width=15, anchor="e").pack(side=tk.LEFT)
        tk.Scale(row1, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.canny_low, showvalue=1).pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        row2 = tk.Frame(param_frame)
        row2.pack(fill=tk.X, padx=5)
        tk.Label(row2, text="Contraste (Haut):", width=15, anchor="e").pack(side=tk.LEFT)
        tk.Scale(row2, from_=0, to=255, orient=tk.HORIZONTAL, variable=self.canny_high, showvalue=1).pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        row3 = tk.Frame(param_frame)
        row3.pack(fill=tk.X, padx=5)
        tk.Label(row3, text="Filtre Poussière:", width=15, anchor="e").pack(side=tk.LEFT)
        tk.Scale(row3, from_=10, to=5000, orient=tk.HORIZONTAL, variable=self.min_area, showvalue=1, resolution=50).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # --- DROITE : Costmap & Éditeur ---
        cost_frame = tk.Frame(self.canvas_frame)
        cost_frame.pack(side=tk.LEFT, padx=20)
        tk.Label(cost_frame, text=f"Costmap avec Overlay ({GRID_W}x{GRID_H} cases)", font=("Arial", 11, "bold")).pack()
        
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
        
        # NOUVEAU : Le curseur d'Overlay (Opacité)
        tk.Label(bottom_right, text="  |  Transparence Costmap :").pack(side=tk.LEFT, padx=(10, 0))
        # command=lambda v: ... permet d'appeler la fonction de mise à jour dès qu'on bouge le slider !
        tk.Scale(bottom_right, from_=0, to=100, orient=tk.HORIZONTAL, variable=self.overlay_alpha, length=120, showvalue=0, command=lambda v: self.update_costmap_canvas()).pack(side=tk.LEFT)

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
        self.lbl_status.config(text="Image redressée ! Cliquez sur Générer Costmap (Ajustez les curseurs si besoin).")

    def process_image(self):
        if self.warped_img is None: 
            messagebox.showerror("Erreur", "Redressez l'image d'abord.")
            return
            
        gray = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        
        low_thresh = self.canny_low.get()
        high_thresh = self.canny_high.get()
        edges = cv2.Canny(blurred, low_thresh, high_thresh)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        binary_obstacles = np.zeros_like(gray)
        min_a = self.min_area.get()
        
        obstacles_found = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_a:
                cv2.drawContours(binary_obstacles, [cnt], -1, 255, thickness=cv2.FILLED)
                obstacles_found += 1
        
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
        
        self.start_pos = None
        self.end_pos = None
        self.update_costmap_canvas()
        self.lbl_status.configure(text=f"Succès ! L'algorithme a trouvé et détouré {obstacles_found} obstacles majeurs.")

    def update_costmap_canvas(self):
        if self.costmap_grid is None: return
        
        # 1. On prépare la grille de couleurs (Bleu -> Rouge)
        disp_grid_large = cv2.resize(self.costmap_grid, (GRID_W*SCALE_UI, GRID_H*SCALE_UI), interpolation=cv2.INTER_NEAREST)
        
        heatmap_rgb = np.zeros((GRID_H*SCALE_UI, GRID_W*SCALE_UI, 3), dtype=np.uint8)
        heatmap_rgb[:, :, 0] = disp_grid_large       
        heatmap_rgb[:, :, 1] = 0
        heatmap_rgb[:, :, 2] = 255 - disp_grid_large 
        
        # --- NOUVEAU : LA FUSION ALPHA (OVERLAY) ---
        alpha = self.overlay_alpha.get() / 100.0 # Curseur de 0 à 100 divisé par 100
        
        # Si on n'est pas à 100% de Costmap ET qu'on a bien l'image de fond
        if alpha < 1.0 and self.warped_img is not None:
            # On redimensionne l'image de la vraie piste à la taille de l'UI (400x600)
            warped_rgb = cv2.cvtColor(self.warped_img, cv2.COLOR_BGR2RGB)
            warped_resized = cv2.resize(warped_rgb, (GRID_W*SCALE_UI, GRID_H*SCALE_UI), interpolation=cv2.INTER_AREA)
            
            # La fonction magique qui mélange les deux images (Heatmap * Alpha + Fond * (1-Alpha))
            final_img = cv2.addWeighted(heatmap_rgb, alpha, warped_resized, 1.0 - alpha, 0)
        else:
            final_img = heatmap_rgb
            
        img_pil = Image.fromarray(final_img)
        self.photo_cost = ImageTk.PhotoImage(img_pil)
        self.canvas_cost.delete("all")
        self.canvas_cost.create_image(0, 0, image=self.photo_cost, anchor=tk.NW)
        
        # Dessins UI par-dessus (Les A et B restent toujours bien visibles !)
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
            if self.costmap_grid[gy, gx] == 255:
                messagebox.showwarning("Attention", "Impossible de démarrer dans un mur ! (Cost=255)")
                return
            self.start_pos = (gx, gy)
            self.update_costmap_canvas()
            
        elif mode == "end" and event.type == tk.EventType.ButtonPress: 
            if self.costmap_grid[gy, gx] == 255:
                messagebox.showwarning("Attention", "Impossible d'arriver dans un mur ! (Cost=255)")
                return
            self.end_pos = (gx, gy)
            self.update_costmap_canvas()
            
        elif mode == "brush": 
            val = self.brush_cost.get() 
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
        cpp_code += f"// MISSION VEGA SC317 - COSTMAP CANNY CONTOURS\n"
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
            
        messagebox.showinfo("Export Réussi", f"Fichier sauvegardé avec la Costmap Canny dans :\n{export_path}")

if __name__ == "__main__":
    root = tk.Tk()
    app = CostmapApp(root)
    root.mainloop()