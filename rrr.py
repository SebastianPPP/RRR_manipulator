import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.animation import FuncAnimation


class RobotRRR:    
    def __init__(self, L1=1.0, L2=1.0, L3=1.0):
        self.L1 = L1  
        self.L2 = L2  
        self.L3 = L3  
        
    def forward_kinematics(self, theta1, theta2, theta3):
        th1 = np.radians(theta1)
        th2 = np.radians(theta2)
        th3 = np.radians(theta3)
        
        # Baza
        x0, y0, z0 = 0, 0, 0
        
        # Pozycja drugiego przegubu
        x1 = 0
        y1 = 0
        z1 = self.L1
        
        # Pozycja trzeciego przegubu
        x2 = self.L2 * np.cos(th2) * np.cos(th1)
        y2 = self.L2 * np.cos(th2) * np.sin(th1)
        z2 = self.L1 + self.L2 * np.sin(th2)
        
        # Pozycja efektora
        x3 = self.L2 * np.cos(th2) * np.cos(th1) + self.L3 * np.cos(th2 + th3) * np.cos(th1)
        y3 = self.L2 * np.cos(th2) * np.sin(th1) + self.L3 * np.cos(th2 + th3) * np.sin(th1)
        z3 = self.L1 + self.L2 * np.sin(th2) + self.L3 * np.sin(th2 + th3)
        
        positions = np.array([
            [x0, y0, z0],
            [x1, y1, z1],
            [x2, y2, z2],
            [x3, y3, z3]
        ])
        
        return positions, (x3, y3, z3)
    
    def inverse_kinematics(self, x, y, z, elbow_up=False):
        theta1 = np.arctan2(y, x)
        
        r = np.sqrt(x**2 + y**2)
        z_rel = z - self.L1
        
        d = np.sqrt(r**2 + z_rel**2)
        
        if d > (self.L2 + self.L3) or d < abs(self.L2 - self.L3):
            return None
        
        cos_theta3 = (d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        
        # Ograniczenie wartości do zakresu [-1, 1] dla arccos (ważne dla błędów numerycznych)
        cos_theta3 = np.clip(cos_theta3, -1, 1)
        
        # Elbow Up: theta3 ujemne, Elbow Down: theta3 dodatnie
        if elbow_up:
            theta3 = -np.arccos(cos_theta3)
        else:
            theta3 = np.arccos(cos_theta3)
        
        alpha = np.arctan2(z_rel, r)
        beta = np.arctan2(self.L3 * np.sin(theta3), self.L2 + self.L3 * np.cos(theta3))
        
        theta2 = alpha - beta
        
        # Konwersja na stopnie
        return (np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
    
    def generate_trajectory(self, start_angles, end_angles, steps=50):
        trajectory = []
        for i in range(steps):
            t = i / (steps - 1)
            theta1 = start_angles[0] + t * (end_angles[0] - start_angles[0])
            theta2 = start_angles[1] + t * (end_angles[1] - start_angles[1])
            theta3 = start_angles[2] + t * (end_angles[2] - start_angles[2])
            
            positions, _ = self.forward_kinematics(theta1, theta2, theta3)
            trajectory.append(positions)
        
        return trajectory


class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Symulator Robota RRR")
        
        self.root.state('zoomed')
        
        self.root.bind('<Escape>', lambda e: self.root.state('normal'))
        self.root.bind('<F11>', lambda e: self.toggle_fullscreen())
        
        self.fullscreen = False
        
        # Inicjalizacja robota
        self.robot = RobotRRR(L1=1.0, L2=1.0, L3=1.0)
        self.current_angles = [0, 0, 0]
        self.trajectory = None
        self.animation = None
        self.current_frame = 0
        
        self.setup_ui()
        self.update_plot()
    
    def toggle_fullscreen(self):
        self.fullscreen = not self.fullscreen
        self.root.attributes('-fullscreen', self.fullscreen)
    
    def create_collapsible_section(self, parent, title):
        # Główny frame dla sekcji
        section_container = ttk.Frame(parent)
        section_container.pack(fill=tk.X, pady=2)
        
        # Frame z nagłówkiem i przyciskiem
        header_frame = ttk.Frame(section_container)
        header_frame.pack(fill=tk.X)
        
        # Zmienna do śledzenia stanu (rozwinięte/zwinięte)
        is_expanded = tk.BooleanVar(value=True)
        
        toggle_btn = ttk.Button(header_frame, text="−", width=3,
                               command=lambda: self.toggle_section(content_frame, toggle_btn, is_expanded))
        toggle_btn.pack(side=tk.LEFT, padx=2)
        
        ttk.Label(header_frame, text=title, font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=5)
        
        content_frame = ttk.LabelFrame(section_container, padding=5)
        content_frame.pack(fill=tk.X, pady=2)
        
        return content_frame
    
    def toggle_section(self, frame, button, is_expanded):
        if is_expanded.get():
            frame.pack_forget()
            button.config(text="+")
            is_expanded.set(False)
        else:
            frame.pack(fill=tk.X, pady=2)
            button.config(text="−")
            is_expanded.set(True)
        
    def setup_ui(self):
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Lewy panel kontrolny
        control_frame = ttk.LabelFrame(main_frame, text="Panel Sterowania", padding=10)
        control_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        # Prawy panel z wykresem
        plot_frame = ttk.Frame(main_frame)
        plot_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        
        length_frame = self.create_collapsible_section(control_frame, "Długości ogniw")
        
        self.L1_var = tk.DoubleVar(value=1.0)
        self.L2_var = tk.DoubleVar(value=1.0)
        self.L3_var = tk.DoubleVar(value=1.0)
        
        ttk.Label(length_frame, text="L1:").grid(row=0, column=0, sticky="w")
        ttk.Scale(length_frame, from_=0.1, to=2.0, variable=self.L1_var, 
                 orient=tk.HORIZONTAL, command=self.update_lengths).grid(row=0, column=1, sticky="ew")
        self.L1_label = ttk.Label(length_frame, text="1.00", width=5)
        self.L1_label.grid(row=0, column=2)
        
        ttk.Label(length_frame, text="L2:").grid(row=1, column=0, sticky="w")
        ttk.Scale(length_frame, from_=0.1, to=2.0, variable=self.L2_var, 
                 orient=tk.HORIZONTAL, command=self.update_lengths).grid(row=1, column=1, sticky="ew")
        self.L2_label = ttk.Label(length_frame, text="1.00", width=5)
        self.L2_label.grid(row=1, column=2)
        
        ttk.Label(length_frame, text="L3:").grid(row=2, column=0, sticky="w")
        ttk.Scale(length_frame, from_=0.1, to=2.0, variable=self.L3_var, 
                 orient=tk.HORIZONTAL, command=self.update_lengths).grid(row=2, column=1, sticky="ew")
        self.L3_label = ttk.Label(length_frame, text="1.00", width=5)
        self.L3_label.grid(row=2, column=2)
        
        length_frame.columnconfigure(1, weight=1)
        
        limits_frame = self.create_collapsible_section(control_frame, "Ograniczenia Kątów")
        
        # Przechowywanie ograniczeń
        self.theta1_min = tk.IntVar(value=-180)
        self.theta1_max = tk.IntVar(value=180)
        self.theta2_min = tk.IntVar(value=-180)
        self.theta2_max = tk.IntVar(value=180)
        self.theta3_min = tk.IntVar(value=-180)
        self.theta3_max = tk.IntVar(value=180)
        
        # θ1 ograniczenia
        ttk.Label(limits_frame, text="θ1:").grid(row=0, column=0, sticky="w")
        ttk.Label(limits_frame, text="min:").grid(row=0, column=1, sticky="e")
        theta1_min_entry = ttk.Entry(limits_frame, textvariable=self.theta1_min, width=6)
        theta1_min_entry.grid(row=0, column=2, padx=2)
        ttk.Label(limits_frame, text="max:").grid(row=0, column=3, sticky="e")
        theta1_max_entry = ttk.Entry(limits_frame, textvariable=self.theta1_max, width=6)
        theta1_max_entry.grid(row=0, column=4, padx=2)
        
        # θ2 ograniczenia
        ttk.Label(limits_frame, text="θ2:").grid(row=1, column=0, sticky="w")
        ttk.Label(limits_frame, text="min:").grid(row=1, column=1, sticky="e")
        theta2_min_entry = ttk.Entry(limits_frame, textvariable=self.theta2_min, width=6)
        theta2_min_entry.grid(row=1, column=2, padx=2)
        ttk.Label(limits_frame, text="max:").grid(row=1, column=3, sticky="e")
        theta2_max_entry = ttk.Entry(limits_frame, textvariable=self.theta2_max, width=6)
        theta2_max_entry.grid(row=1, column=4, padx=2)
        
        # θ3 ograniczenia
        ttk.Label(limits_frame, text="θ3:").grid(row=2, column=0, sticky="w")
        ttk.Label(limits_frame, text="min:").grid(row=2, column=1, sticky="e")
        theta3_min_entry = ttk.Entry(limits_frame, textvariable=self.theta3_min, width=6)
        theta3_min_entry.grid(row=2, column=2, padx=2)
        ttk.Label(limits_frame, text="max:").grid(row=2, column=3, sticky="e")
        theta3_max_entry = ttk.Entry(limits_frame, textvariable=self.theta3_max, width=6)
        theta3_max_entry.grid(row=2, column=4, padx=2)
        
        ttk.Button(limits_frame, text="Zastosuj Ograniczenia", command=self.apply_limits).grid(
            row=3, column=0, columnspan=5, pady=5, sticky="ew")
        
        # Kinematyka prosta
        fk_frame = self.create_collapsible_section(control_frame, "Kinematyka Prosta")
        
        self.theta1_var = tk.DoubleVar(value=0)
        self.theta2_var = tk.DoubleVar(value=0)
        self.theta3_var = tk.DoubleVar(value=0)
        
        ttk.Label(fk_frame, text="θ1 (°):").grid(row=0, column=0, sticky="w")
        self.theta1_scale = ttk.Scale(fk_frame, from_=-180, to=180, variable=self.theta1_var, 
                 orient=tk.HORIZONTAL, command=self.update_forward)
        self.theta1_scale.grid(row=0, column=1, sticky="ew")
        self.theta1_label = ttk.Label(fk_frame, text="0.00", width=5)
        self.theta1_label.grid(row=0, column=2)
        
        ttk.Label(fk_frame, text="θ2 (°):").grid(row=1, column=0, sticky="w")
        self.theta2_scale = ttk.Scale(fk_frame, from_=-180, to=180, variable=self.theta2_var, 
                 orient=tk.HORIZONTAL, command=self.update_forward)
        self.theta2_scale.grid(row=1, column=1, sticky="ew")
        self.theta2_label = ttk.Label(fk_frame, text="0.00", width=5)
        self.theta2_label.grid(row=1, column=2)
        
        ttk.Label(fk_frame, text="θ3 (°):").grid(row=2, column=0, sticky="w")
        self.theta3_scale = ttk.Scale(fk_frame, from_=-180, to=180, variable=self.theta3_var, 
                 orient=tk.HORIZONTAL, command=self.update_forward)
        self.theta3_scale.grid(row=2, column=1, sticky="ew")
        self.theta3_label = ttk.Label(fk_frame, text="0.00", width=5)
        self.theta3_label.grid(row=2, column=2)
        
        fk_frame.columnconfigure(1, weight=1)
        
        # Wynik kinematyki prostej
        self.fk_result = tk.StringVar(value="Pozycja: (0.00, 0.00, 0.00)")
        ttk.Label(fk_frame, textvariable=self.fk_result, font=("Arial", 9, "bold")).grid(
            row=3, column=0, columnspan=3, pady=5)
        
        # Kinematyka odwrotna
        ik_frame = self.create_collapsible_section(control_frame, "Kinematyka Odwrotna")
        
        ttk.Label(ik_frame, text="X:").grid(row=0, column=0, sticky="w")
        self.x_entry = ttk.Entry(ik_frame, width=10)
        self.x_entry.grid(row=0, column=1, sticky="ew", padx=2)
        self.x_entry.insert(0, "1.0")
        
        ttk.Label(ik_frame, text="Y:").grid(row=1, column=0, sticky="w")
        self.y_entry = ttk.Entry(ik_frame, width=10)
        self.y_entry.grid(row=1, column=1, sticky="ew", padx=2)
        self.y_entry.insert(0, "1.0")
        
        ttk.Label(ik_frame, text="Z:").grid(row=2, column=0, sticky="w")
        self.z_entry = ttk.Entry(ik_frame, width=10)
        self.z_entry.grid(row=2, column=1, sticky="ew", padx=2)
        self.z_entry.insert(0, "2.0")
        
        # Opcja Elbow Up/Down
        ttk.Label(ik_frame, text="Konfiguracja:").grid(row=3, column=0, sticky="w")
        self.elbow_config = tk.StringVar(value="Elbow Down")
        elbow_combo = ttk.Combobox(ik_frame, textvariable=self.elbow_config, 
                                    values=["Elbow Up", "Elbow Down"], state="readonly", width=12)
        elbow_combo.grid(row=3, column=1, sticky="ew", padx=2, pady=2)
        
        ttk.Button(ik_frame, text="Oblicz IK", command=self.calculate_inverse).grid(
            row=4, column=0, columnspan=2, pady=5, sticky="ew")
        
        self.ik_result = tk.StringVar(value="Kąty: -")
        ttk.Label(ik_frame, textvariable=self.ik_result, font=("Arial", 9, "bold"), 
                 wraplength=200).grid(row=5, column=0, columnspan=2, pady=5)
        
        ik_frame.columnconfigure(1, weight=1)
        
        # Trajektoria
        traj_frame = self.create_collapsible_section(control_frame, "Animacja Trajektorii")
        
        ttk.Label(traj_frame, text="Kąty docelowe:").grid(row=0, column=0, columnspan=2, sticky="w")
        
        ttk.Label(traj_frame, text="θ1:").grid(row=1, column=0, sticky="w")
        self.target_theta1 = ttk.Entry(traj_frame, width=10)
        self.target_theta1.grid(row=1, column=1, sticky="ew", padx=2)
        self.target_theta1.insert(0, "90")
        
        ttk.Label(traj_frame, text="θ2:").grid(row=2, column=0, sticky="w")
        self.target_theta2 = ttk.Entry(traj_frame, width=10)
        self.target_theta2.grid(row=2, column=1, sticky="ew", padx=2)
        self.target_theta2.insert(0, "45")
        
        ttk.Label(traj_frame, text="θ3:").grid(row=3, column=0, sticky="w")
        self.target_theta3 = ttk.Entry(traj_frame, width=10)
        self.target_theta3.grid(row=3, column=1, sticky="ew", padx=2)
        self.target_theta3.insert(0, "30")
        
        ttk.Button(traj_frame, text="Start Animacji", command=self.start_animation).grid(
            row=4, column=0, columnspan=2, pady=5, sticky="ew")
        
        ttk.Button(traj_frame, text="Stop Animacji", command=self.stop_animation).grid(
            row=5, column=0, columnspan=2, pady=5, sticky="ew")
        
        traj_frame.columnconfigure(1, weight=1)
        
        # Współrzędne przegubów
        coords_frame = self.create_collapsible_section(control_frame, "Współrzędne Przegubów")
        
        # Nagłówki
        ttk.Label(coords_frame, text="Przegub", font=("Arial", 9, "bold")).grid(row=0, column=0, sticky="w", padx=5)
        ttk.Label(coords_frame, text="X", font=("Arial", 9, "bold")).grid(row=0, column=1, sticky="w", padx=5)
        ttk.Label(coords_frame, text="Y", font=("Arial", 9, "bold")).grid(row=0, column=2, sticky="w", padx=5)
        ttk.Label(coords_frame, text="Z", font=("Arial", 9, "bold")).grid(row=0, column=3, sticky="w", padx=5)
        
        # Podstawa
        ttk.Label(coords_frame, text="Podstawa:").grid(row=1, column=0, sticky="w", padx=5)
        self.base_x = ttk.Label(coords_frame, text="0.00")
        self.base_x.grid(row=1, column=1, sticky="w", padx=5)
        self.base_y = ttk.Label(coords_frame, text="0.00")
        self.base_y.grid(row=1, column=2, sticky="w", padx=5)
        self.base_z = ttk.Label(coords_frame, text="0.00")
        self.base_z.grid(row=1, column=3, sticky="w", padx=5)
        
        # Przegub 1
        ttk.Label(coords_frame, text="Przegub 1:").grid(row=2, column=0, sticky="w", padx=5)
        self.joint1_x = ttk.Label(coords_frame, text="0.00")
        self.joint1_x.grid(row=2, column=1, sticky="w", padx=5)
        self.joint1_y = ttk.Label(coords_frame, text="0.00")
        self.joint1_y.grid(row=2, column=2, sticky="w", padx=5)
        self.joint1_z = ttk.Label(coords_frame, text="0.00")
        self.joint1_z.grid(row=2, column=3, sticky="w", padx=5)
        
        # Przegub 2
        ttk.Label(coords_frame, text="Przegub 2:").grid(row=3, column=0, sticky="w", padx=5)
        self.joint2_x = ttk.Label(coords_frame, text="0.00")
        self.joint2_x.grid(row=3, column=1, sticky="w", padx=5)
        self.joint2_y = ttk.Label(coords_frame, text="0.00")
        self.joint2_y.grid(row=3, column=2, sticky="w", padx=5)
        self.joint2_z = ttk.Label(coords_frame, text="0.00")
        self.joint2_z.grid(row=3, column=3, sticky="w", padx=5)
        
        # Efektor
        ttk.Label(coords_frame, text="Efektor:", font=("Arial", 9, "bold")).grid(row=4, column=0, sticky="w", padx=5)
        self.effector_x = ttk.Label(coords_frame, text="0.00", font=("Arial", 9, "bold"))
        self.effector_x.grid(row=4, column=1, sticky="w", padx=5)
        self.effector_y = ttk.Label(coords_frame, text="0.00", font=("Arial", 9, "bold"))
        self.effector_y.grid(row=4, column=2, sticky="w", padx=5)
        self.effector_z = ttk.Label(coords_frame, text="0.00", font=("Arial", 9, "bold"))
        self.effector_z.grid(row=4, column=3, sticky="w", padx=5)
        
        # Wykres 3D
        self.fig = Figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def update_lengths(self, event=None):
        self.robot.L1 = self.L1_var.get()
        self.robot.L2 = self.L2_var.get()
        self.robot.L3 = self.L3_var.get()
        
        self.L1_label.config(text=f"{self.L1_var.get():.2f}")
        self.L2_label.config(text=f"{self.L2_var.get():.2f}")
        self.L3_label.config(text=f"{self.L3_var.get():.2f}")
        
        self.update_plot()
    
    def apply_limits(self):
        try:
            # Wartości ograniczeń
            theta1_min = self.theta1_min.get()
            theta1_max = self.theta1_max.get()
            theta2_min = self.theta2_min.get()
            theta2_max = self.theta2_max.get()
            theta3_min = self.theta3_min.get()
            theta3_max = self.theta3_max.get()
            
            # Walidacja
            if theta1_min >= theta1_max or theta2_min >= theta2_max or theta3_min >= theta3_max:
                messagebox.showerror("Błąd", "Wartość minimalna musi być mniejsza niż maksymalna!")
                return
            
            self.theta1_scale.config(from_=theta1_min, to=theta1_max)
            self.theta2_scale.config(from_=theta2_min, to=theta2_max)
            self.theta3_scale.config(from_=theta3_min, to=theta3_max)
            
            if self.theta1_var.get() < theta1_min:
                self.theta1_var.set(theta1_min)
            elif self.theta1_var.get() > theta1_max:
                self.theta1_var.set(theta1_max)
                
            if self.theta2_var.get() < theta2_min:
                self.theta2_var.set(theta2_min)
            elif self.theta2_var.get() > theta2_max:
                self.theta2_var.set(theta2_max)
                
            if self.theta3_var.get() < theta3_min:
                self.theta3_var.set(theta3_min)
            elif self.theta3_var.get() > theta3_max:
                self.theta3_var.set(theta3_max)
            
            self.update_forward()
            messagebox.showinfo("Sukces", "Ograniczenia kątów zostały zastosowane!")
            
        except tk.TclError:
            messagebox.showerror("Błąd", "Wprowadź poprawne wartości liczbowe dla ograniczeń!")
        
    def update_forward(self, event=None):
        self.current_angles = [
            self.theta1_var.get(),
            self.theta2_var.get(),
            self.theta3_var.get()
        ]
        
        self.theta1_label.config(text=f"{self.theta1_var.get():.2f}")
        self.theta2_label.config(text=f"{self.theta2_var.get():.2f}")
        self.theta3_label.config(text=f"{self.theta3_var.get():.2f}")
        
        self.update_plot()
        
    def update_plot(self):
        self.ax.clear()
        
        # Obliczenie pozycji
        positions, end_pos = self.robot.forward_kinematics(*self.current_angles)
        
        # Aktualizacja współrzędnych przegubów
        self.base_x.config(text=f"{positions[0, 0]:.2f}")
        self.base_y.config(text=f"{positions[0, 1]:.2f}")
        self.base_z.config(text=f"{positions[0, 2]:.2f}")
        
        self.joint1_x.config(text=f"{positions[1, 0]:.2f}")
        self.joint1_y.config(text=f"{positions[1, 1]:.2f}")
        self.joint1_z.config(text=f"{positions[1, 2]:.2f}")
        
        self.joint2_x.config(text=f"{positions[2, 0]:.2f}")
        self.joint2_y.config(text=f"{positions[2, 1]:.2f}")
        self.joint2_z.config(text=f"{positions[2, 2]:.2f}")
        
        self.effector_x.config(text=f"{positions[3, 0]:.2f}")
        self.effector_y.config(text=f"{positions[3, 1]:.2f}")
        self.effector_z.config(text=f"{positions[3, 2]:.2f}")
        
        # Rysowanie robota
        self.ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                    'o-', linewidth=3, markersize=8, color='blue', label='Robot')
        
        # Rysowanie podstawy
        self.ax.plot([0], [0], [0], 'rs', markersize=12, label='Podstawa')
        
        # Rysowanie efektora
        self.ax.plot([end_pos[0]], [end_pos[1]], [end_pos[2]], 
                    'g*', markersize=15, label='Efektor')
        
        # Ustawienia wykresu
        max_reach = self.robot.L1 + self.robot.L2 + self.robot.L3
        self.ax.set_xlim([-max_reach, max_reach])
        self.ax.set_ylim([-max_reach, max_reach])
        self.ax.set_zlim([0, max_reach * 1.2])
        
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Robot RRR')
        self.ax.legend()
        self.ax.grid(True)
        
        # Aktualizacja wyniku kinematyki prostej
        self.fk_result.set(f"Pozycja: ({end_pos[0]:.2f}, {end_pos[1]:.2f}, {end_pos[2]:.2f})")
        
        self.canvas.draw()
        
    def calculate_inverse(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            
            # Sprawdź konfigurację
            elbow_up = (self.elbow_config.get() == "Elbow Up")
            
            result = self.robot.inverse_kinematics(x, y, z, elbow_up=elbow_up)
            if result is None:
                self.ik_result.set("Pozycja nieosiągalna!")
                messagebox.showwarning("Błąd", "Podana pozycja jest poza zasięgiem robota!")
                return
            theta1, theta2, theta3 = result
            # Sprawdź ograniczenia kątów
            if not (self.theta1_min.get() <= theta1 <= self.theta1_max.get() and
                    self.theta2_min.get() <= theta2 <= self.theta2_max.get() and
                    self.theta3_min.get() <= theta3 <= self.theta3_max.get()):
                self.ik_result.set("Pozycja nieosiągalna przez ograniczenia!")
                messagebox.showwarning("Błąd", "Pozycja wykracza poza ograniczenia kątów!")
                return
            self.ik_result.set(f"Kąty:\nθ1 = {theta1:.2f}°\nθ2 = {theta2:.2f}°\nθ3 = {theta3:.2f}°")
            
            # Zapisz kąty docelowe dla animacji
            self.animation_target_angles = [theta1, theta2, theta3]
            
            target_angles = [theta1, theta2, theta3]
            self.trajectory = self.robot.generate_trajectory(
                self.current_angles, target_angles, steps=50
            )
            self.current_frame = 0
            if self.animation is not None:
                try:
                    self.animation.event_source.stop()
                except:
                    pass
                self.animation = None
            self.animation = FuncAnimation(
                self.fig, self.animate, frames=len(self.trajectory),
                interval=50, repeat=False, blit=False
            )
            self.canvas.draw()
        except ValueError:
            messagebox.showerror("Błąd", "Wprowadź poprawne wartości liczbowe!")
            
    def start_animation(self):
        try:
            target_angles = [
                float(self.target_theta1.get()),
                float(self.target_theta2.get()),
                float(self.target_theta3.get())
            ]
            
            self.animation_target_angles = target_angles
            
            # Generowanie trajektorii
            self.trajectory = self.robot.generate_trajectory(
                self.current_angles, target_angles, steps=50
            )
            self.current_frame = 0
            
            # Zatrzymanie i wyczyszczenie poprzedniej animacji
            if self.animation is not None:
                try:
                    self.animation.event_source.stop()
                except:
                    pass
                self.animation = None
            
            # Utworzenie nowej animacji
            self.animation = FuncAnimation(
                self.fig, self.animate, frames=len(self.trajectory),
                interval=50, repeat=False, blit=False
            )
            self.canvas.draw()
            
        except ValueError:
            messagebox.showerror("Błąd", "Wprowadź poprawne wartości kątów!")
        except Exception as e:
            messagebox.showerror("Błąd", f"Wystąpił błąd: {str(e)}")
            
    def stop_animation(self):
        if self.animation is not None:
            try:
                self.animation.event_source.stop()
            except:
                pass
            self.animation = None
            
    def animate(self, frame):
        if frame >= len(self.trajectory):
            return
        
        self.ax.clear()
        positions = self.trajectory[frame]
        
        self.base_x.config(text=f"{positions[0, 0]:.2f}")
        self.base_y.config(text=f"{positions[0, 1]:.2f}")
        self.base_z.config(text=f"{positions[0, 2]:.2f}")
        
        self.joint1_x.config(text=f"{positions[1, 0]:.2f}")
        self.joint1_y.config(text=f"{positions[1, 1]:.2f}")
        self.joint1_z.config(text=f"{positions[1, 2]:.2f}")
        
        self.joint2_x.config(text=f"{positions[2, 0]:.2f}")
        self.joint2_y.config(text=f"{positions[2, 1]:.2f}")
        self.joint2_z.config(text=f"{positions[2, 2]:.2f}")
        
        self.effector_x.config(text=f"{positions[3, 0]:.2f}")
        self.effector_y.config(text=f"{positions[3, 1]:.2f}")
        self.effector_z.config(text=f"{positions[3, 2]:.2f}")
        
        # Rysowanie robota w bieżącej pozycji
        self.ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                    'o-', linewidth=3, markersize=8, color='blue', label='Robot')
        
        # Rysowanie śladu trajektorii
        if frame > 0:
            trail = np.array([self.trajectory[i][-1] for i in range(frame + 1)])
            self.ax.plot(trail[:, 0], trail[:, 1], trail[:, 2], 
                        'r--', linewidth=1, alpha=0.5, label='Trajektoria')
        
        # Podstawa i efektor
        self.ax.plot([0], [0], [0], 'rs', markersize=12, label='Podstawa')
        self.ax.plot([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]], 
                    'g*', markersize=15, label='Efektor')
        
        # Ustawienia
        max_reach = self.robot.L1 + self.robot.L2 + self.robot.L3
        self.ax.set_xlim([-max_reach, max_reach])
        self.ax.set_ylim([-max_reach, max_reach])
        self.ax.set_zlim([0, max_reach * 1.2])
        
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title(f'Robot RRR - Animacja Trajektorii')
        self.ax.legend()
        self.ax.grid(True)
        
        if frame == len(self.trajectory) - 1:
            if hasattr(self, 'animation_target_angles'):
                target_angles = self.animation_target_angles
            else:
                target_angles = [
                    float(self.target_theta1.get()),
                    float(self.target_theta2.get()),
                    float(self.target_theta3.get())
                ]
            
            self.current_angles = target_angles
            self.theta1_var.set(target_angles[0])
            self.theta2_var.set(target_angles[1])
            self.theta3_var.set(target_angles[2])
            
            self.theta1_label.config(text=f"{target_angles[0]:.2f}")
            self.theta2_label.config(text=f"{target_angles[1]:.2f}")
            self.theta3_label.config(text=f"{target_angles[2]:.2f}")


def main():
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
