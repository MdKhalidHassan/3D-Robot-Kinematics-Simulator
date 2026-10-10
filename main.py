"""
=============================================================================
3D Robot Kinematics Simulator (Clean Transparent Grid Edition)
=============================================================================
Description : A 3D Forward Kinematics engine and real-time simulator featuring
              vibrant day-light clarity, scrollable UI, and clean grid workspace.
License     : MIT
Language    : Python 3
=============================================================================
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk

MAX_JOINTS = 6
LINK_COLORS = ['#FF5733', '#3498DB', '#2ECC71', '#9B59B6', '#F1C40F', '#E67E22']


class IndustrialRobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("3D Robot Kinematics Simulator — Clean Transparent Workspace")
        self.root.geometry("1480x900")

        # Initial Robot & Camera Configurations
        self.n_joints = 3
        self.base_height = 50.0   # Adjustable Base Height from Ground (mm)
        self.link_radius = 14.0   # Adjustable Global Link Thickness (Radius in mm)
        self.cam_azim = -60.0     # Camera Horizontal Rotation (Azimuth)
        self.cam_elev = 30.0      # Camera Vertical Tilt (Elevation)
        
        self.joint_modes = ["R_YAW", "R_PITCH", "R_PITCH", "R_YAW", "R_PITCH", "R_YAW"]
        self.link_lengths = [100.0, 120.0, 100.0, 80.0, 60.0, 40.0]
        self.joint_values = [30.0, 45.0, -30.0, 0.0, 0.0, 0.0]
        self.joint_offsets = [0.0] * MAX_JOINTS  # Cumulative Zero Reference Offsets

        self.setup_ui()
        self.update_simulation()

    def setup_ui(self):
        # Main Layout Setup
        self.control_frame = ttk.LabelFrame(self.root, text=" Control, Camera & Teach Dashboard ", padding=10)
        self.control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        self.view_frame = ttk.LabelFrame(self.root, text=" Real-Time 3D Industrial View ", padding=5)
        self.view_frame.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH, padx=10, pady=10)

        # 1. Global Setup (DOF, Base Height, Link Thickness)
        global_frame = ttk.LabelFrame(self.control_frame, text=" Global Setup ", padding=5)
        global_frame.pack(fill=tk.X, pady=3)

        # DOF Selector
        dof_box = ttk.Frame(global_frame)
        dof_box.pack(fill=tk.X, pady=2)
        ttk.Label(dof_box, text="Active DOF: ", font=('Helvetica', 9, 'bold')).pack(side=tk.LEFT)
        self.dof_var = tk.IntVar(value=self.n_joints)
        dof_spin = ttk.Spinbox(dof_box, from_=1, to=MAX_JOINTS, textvariable=self.dof_var, width=5, command=self.on_dof_change)
        dof_spin.pack(side=tk.LEFT, padx=5)

        # Base Height Slider
        base_box = ttk.Frame(global_frame)
        base_box.pack(fill=tk.X, pady=2)
        ttk.Label(base_box, text="Base Height (mm):", font=('Helvetica', 9, 'bold')).pack(side=tk.LEFT)
        
        self.base_ent = ttk.Entry(base_box, width=6)
        self.base_ent.insert(0, f"{self.base_height:.0f}")
        self.base_ent.pack(side=tk.RIGHT, padx=2)
        self.base_ent.bind("<Return>", self.on_base_entry_change)

        self.base_sld = ttk.Scale(base_box, from_=0.0, to=300.0, value=self.base_height, command=self.on_base_slider_change)
        self.base_sld.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Link Thickness (Radius) Slider
        thick_box = ttk.Frame(global_frame)
        thick_box.pack(fill=tk.X, pady=2)
        ttk.Label(thick_box, text="Link Radius (mm):", font=('Helvetica', 9, 'bold')).pack(side=tk.LEFT)
        
        self.thick_ent = ttk.Entry(thick_box, width=6)
        self.thick_ent.insert(0, f"{self.link_radius:.0f}")
        self.thick_ent.pack(side=tk.RIGHT, padx=2)
        self.thick_ent.bind("<Return>", self.on_thick_entry_change)

        self.thick_sld = ttk.Scale(thick_box, from_=5.0, to=30.0, value=self.link_radius, command=self.on_thick_slider_change)
        self.thick_sld.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # 2. Camera Viewport Controls (Azimuth & Elevation)
        cam_frame = ttk.LabelFrame(self.control_frame, text=" Camera Viewport Control ", padding=5)
        cam_frame.pack(fill=tk.X, pady=3)

        # Azimuth (Horizontal Rotation)
        azim_box = ttk.Frame(cam_frame)
        azim_box.pack(fill=tk.X, pady=2)
        ttk.Label(azim_box, text="Azimuth (Yaw °):", font=('Helvetica', 9, 'bold')).pack(side=tk.LEFT)
        
        self.azim_ent = ttk.Entry(azim_box, width=6)
        self.azim_ent.insert(0, f"{self.cam_azim:.0f}")
        self.azim_ent.pack(side=tk.RIGHT, padx=2)
        self.azim_ent.bind("<Return>", self.on_azim_entry_change)

        self.azim_sld = ttk.Scale(azim_box, from_=-180.0, to=180.0, value=self.cam_azim, command=self.on_azim_slider_change)
        self.azim_sld.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # Elevation (Vertical Tilt)
        elev_box = ttk.Frame(cam_frame)
        elev_box.pack(fill=tk.X, pady=2)
        ttk.Label(elev_box, text="Elevation (Pitch °):", font=('Helvetica', 9, 'bold')).pack(side=tk.LEFT)
        
        self.elev_ent = ttk.Entry(elev_box, width=6)
        self.elev_ent.insert(0, f"{self.cam_elev:.0f}")
        self.elev_ent.pack(side=tk.RIGHT, padx=2)
        self.elev_ent.bind("<Return>", self.on_elev_entry_change)

        self.elev_sld = ttk.Scale(elev_box, from_=-90.0, to=90.0, value=self.cam_elev, command=self.on_elev_slider_change)
        self.elev_sld.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        # 3. SCROLLABLE CONTAINER FOR JOINT CONTROLS
        scroll_outer_frame = ttk.Frame(self.control_frame)
        scroll_outer_frame.pack(fill=tk.BOTH, expand=True, pady=3)

        self.canvas_joints = tk.Canvas(scroll_outer_frame, borderwidth=0, highlightthickness=0, width=380)
        self.scrollbar_joints = ttk.Scrollbar(scroll_outer_frame, orient="vertical", command=self.canvas_joints.yview)
        
        self.joint_controls_container = ttk.Frame(self.canvas_joints)
        self.joint_controls_container.bind(
            "<Configure>",
            lambda event: self.canvas_joints.configure(scrollregion=self.canvas_joints.bbox("all"))
        )

        self.canvas_joints.create_window((0, 0), window=self.joint_controls_container, anchor="nw")
        self.canvas_joints.configure(yscrollcommand=self.scrollbar_joints.set)

        self.canvas_joints.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar_joints.pack(side=tk.RIGHT, fill=tk.Y)

        def _on_mousewheel(event):
            self.canvas_joints.yview_scroll(int(-1*(event.delta/120)), "units")
        self.canvas_joints.bind_all("<MouseWheel>", _on_mousewheel)

        # 4. Output Data Display Box
        self.output_frame = ttk.LabelFrame(self.control_frame, text=" Live Output Coordinates ", padding=10)
        self.output_frame.pack(fill=tk.X, pady=5, side=tk.BOTTOM)

        self.lbl_ee_x = ttk.Label(self.output_frame, text="End-Effector X: 0.00 mm", font=('Consolas', 10, 'bold'), foreground="#d32f2f")
        self.lbl_ee_x.pack(anchor=tk.W)
        self.lbl_ee_y = ttk.Label(self.output_frame, text="End-Effector Y: 0.00 mm", font=('Consolas', 10, 'bold'), foreground="#388e3c")
        self.lbl_ee_y.pack(anchor=tk.W)
        self.lbl_ee_z = ttk.Label(self.output_frame, text="End-Effector Z: 0.00 mm (Height)", font=('Consolas', 10, 'bold'), foreground="#1976d2")
        self.lbl_ee_z.pack(anchor=tk.W)

        # 5. Matplotlib 3D Canvas
        self.fig = plt.figure(figsize=(7, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.view_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.rebuild_joint_inputs()

    def rebuild_joint_inputs(self):
        for widget in self.joint_controls_container.winfo_children():
            widget.destroy()

        self.val_sliders = []
        self.val_entries = []
        self.len_entries = []

        for i in range(self.n_joints):
            j_frame = ttk.LabelFrame(self.joint_controls_container, text=f" Joint {i+1} Configuration ", padding=5)
            j_frame.pack(fill=tk.X, pady=3, padx=2)

            row1 = ttk.Frame(j_frame)
            row1.pack(fill=tk.X, pady=1)
            ttk.Label(row1, text="Motion Axis:").pack(side=tk.LEFT)
            mode_combo = ttk.Combobox(row1, values=["Revolute - Horizontal (Yaw)", "Revolute - Vertical (Pitch)", "Prismatic (P)"], state="readonly", width=22)
            
            if self.joint_modes[i] == "R_YAW":
                mode_combo.set("Revolute - Horizontal (Yaw)")
            elif self.joint_modes[i] == "R_PITCH":
                mode_combo.set("Revolute - Vertical (Pitch)")
            else:
                mode_combo.set("Prismatic (P)")

            mode_combo.pack(side=tk.LEFT, padx=5)
            mode_combo.bind("<<ComboboxSelected>>", lambda e, idx=i, cb=mode_combo: self.on_mode_change(idx, cb.get()))

            row2 = ttk.Frame(j_frame)
            row2.pack(fill=tk.X, pady=1)
            ttk.Label(row2, text="L (mm):").pack(side=tk.LEFT)
            len_ent = ttk.Entry(row2, width=7)
            len_ent.insert(0, str(self.link_lengths[i]))
            len_ent.pack(side=tk.LEFT, padx=2)
            len_ent.bind("<Return>", lambda e, idx=i, ent=len_ent: self.on_len_entry_change(idx, ent.get()))
            self.len_entries.append(len_ent)

            set_zero_btn = ttk.Button(row2, text="Set as Zero", command=lambda idx=i: self.set_current_as_zero(idx))
            set_zero_btn.pack(side=tk.RIGHT, padx=2)

            row3 = ttk.Frame(j_frame)
            row3.pack(fill=tk.X, pady=1)
            unit_lbl = "Ext d:" if self.joint_modes[i] == "P" else "Theta:"
            ttk.Label(row3, text=f"{unit_lbl}").pack(side=tk.LEFT)
            
            val_ent = ttk.Entry(row3, width=7)
            val_ent.insert(0, f"{self.joint_values[i]:.1f}")
            val_ent.pack(side=tk.RIGHT, padx=2)
            val_ent.bind("<Return>", lambda e, idx=i, ent=val_ent: self.on_val_entry_change(idx, ent.get()))
            self.val_entries.append(val_ent)

            min_v, max_v = (0.0, 150.0) if self.joint_modes[i] == "P" else (-180.0, 180.0)
            val_sld = ttk.Scale(row3, from_=min_v, to=max_v, value=self.joint_values[i], 
                                command=lambda val, idx=i: self.on_slider_change(idx, val))
            val_sld.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.val_sliders.append(val_sld)

    def draw_solid_cylinder(self, p1, p2, radius=14, color='#3498DB'):
        p1 = np.asarray(p1, dtype=np.float64)
        p2 = np.asarray(p2, dtype=np.float64)
        v = p2 - p1
        mag = np.linalg.norm(v)
        if mag < 1e-3: return
        v = v / mag
        not_v = np.array([1.0, 0.0, 0.0]) if abs(v[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        n1 = np.cross(v, not_v); n1 = n1 / np.linalg.norm(n1)
        n2 = np.cross(v, n1)

        t = np.linspace(0, 2 * np.pi, 16)
        r = np.linspace(0, mag, 2)
        t, r = np.meshgrid(t, r)

        X = p1[0] + r * v[0] + radius * np.cos(t) * n1[0] + radius * np.sin(t) * n2[0]
        Y = p1[1] + r * v[1] + radius * np.cos(t) * n1[1] + radius * np.sin(t) * n2[1]
        Z = p1[2] + r * v[2] + radius * np.cos(t) * n1[2] + radius * np.sin(t) * n2[2]

        self.ax.plot_surface(X, Y, Z, color=color, alpha=1.0, shade=False, zorder=5)

    def draw_solid_sphere(self, center, radius=18, color='#2C3E50'):
        center = np.asarray(center, dtype=np.float64)
        u = np.linspace(0, 2 * np.pi, 16)
        v = np.linspace(0, np.pi, 16)
        X = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        Y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        Z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        self.ax.plot_surface(X, Y, Z, color=color, alpha=1.0, shade=False, zorder=6)

    def draw_transparent_grid_floor(self, size=300):
        """
        Renders the gorgeous crystal clear transparent grid workspace at Z = 0.
        """
        step = 50
        x = np.arange(-size, size + step, step)
        y = np.arange(-size, size + step, step)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)

        # Transparent blue overlay floor
        self.ax.plot_surface(X, Y, Z, color='#2980b9', alpha=0.15, shade=False, zorder=0)

        # Crisp grid lines
        for g in range(-size, size + 1, step):
            self.ax.plot([-size, size], [g, g], [0, 0], color='#2980b9', linestyle='-', linewidth=0.6, alpha=0.6, zorder=1)
            self.ax.plot([g, g], [-size, size], [0, 0], color='#2980b9', linestyle='-', linewidth=0.6, alpha=0.6, zorder=1)
        
        # Perimeter Border
        self.ax.plot([-size, size, size, -size, -size], [-size, -size, size, size, -size], [0,0,0,0,0], color='#1b4f72', linewidth=1.5, alpha=0.9, zorder=2)

    def calculate_fk(self):
        points = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
        T = np.eye(4, dtype=np.float64)

        for i in range(self.n_joints):
            mode = self.joint_modes[i]
            val = float(self.joint_values[i]) + float(self.joint_offsets[i])
            l_len = float(self.link_lengths[i])

            if mode == "R_YAW":
                rad = np.deg2rad(val)
                cos_a, sin_a = np.cos(rad), np.sin(rad)
                R_z = np.array([
                    [cos_a, -sin_a, 0, 0],
                    [sin_a,  cos_a, 0, 0],
                    [0,          0, 1, 0],
                    [0,          0, 0, 1]
                ], dtype=np.float64)
                Trans = np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, l_len],
                    [0, 0, 0, 1]
                ], dtype=np.float64)
                T_local = R_z @ Trans

            elif mode == "R_PITCH":
                rad = np.deg2rad(val)
                cos_a, sin_a = np.cos(rad), np.sin(rad)
                R_y = np.array([
                    [ cos_a, 0, sin_a, 0],
                    [     0, 1,     0, 0],
                    [-sin_a, 0, cos_a, 0],
                    [     0, 0,     0, 1]
                ], dtype=np.float64)
                Trans = np.array([
                    [1, 0, 0, l_len],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ], dtype=np.float64)
                T_local = R_y @ Trans

            else:  # Prismatic
                dist = l_len + val
                T_local = np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, dist],
                    [0, 0, 0, 1]
                ], dtype=np.float64)

            T = T @ T_local
            pos = T[:3, 3]
            points.append(pos)

        return points

    def update_simulation(self):
        self.ax.clear()

        points = self.calculate_fk()

        reach = sum(self.link_lengths[:self.n_joints]) + 100
        grid_size = int(max(250, reach))
        
        # Draw Clean Transparent Workspace Grid
        self.draw_transparent_grid_floor(size=grid_size)

        b_height = float(self.base_height)
        r_thick = float(self.link_radius)
        pedestal_top = np.array([0.0, 0.0, max(5.0, b_height)], dtype=np.float64)
        
        self.draw_solid_cylinder(np.array([0.0, 0.0, 0.0]), pedestal_top, radius=r_thick * 2.2, color='#34495E')
        self.draw_solid_sphere(pedestal_top, radius=r_thick * 1.4, color='#1B2631')

        adj_points = [p + pedestal_top for p in points]

        for i in range(self.n_joints):
            p1, p2 = adj_points[i], adj_points[i+1]
            color = LINK_COLORS[i % len(LINK_COLORS)]
            
            self.draw_solid_cylinder(p1, p2, radius=r_thick, color=color)
            j_color = '#222222' if i < self.n_joints - 1 else '#C0392B'
            self.draw_solid_sphere(p2, radius=r_thick * 1.2, color=j_color)

            mid_p = (p1 + p2) / 2.0
            eff_val = self.joint_values[i] + self.joint_offsets[i]
            var_sym = f"d{i+1}={eff_val:.0f}mm" if self.joint_modes[i] == "P" else f"θ{i+1}={eff_val:.0f}°"
            label_text = f" J{i+1} ({var_sym})"
            self.ax.text(mid_p[0], mid_p[1], mid_p[2] + 12, label_text, color='#000000', fontsize=8, fontweight='bold', zorder=10)

        ee = adj_points[-1]
        self.ax.plot([ee[0]-15, ee[0]+15], [ee[1], ee[1]], [ee[2], ee[2]], color='red', linewidth=2, zorder=8)
        self.ax.plot([ee[0], ee[0]], [ee[1]-15, ee[1]+15], [ee[2], ee[2]], color='red', linewidth=2, zorder=8)
        self.ax.plot([ee[0], ee[0]], [ee[1], ee[1]], [ee[2]-15, ee[2]+15], color='red', linewidth=2, zorder=8)

        lim = max(220, reach)
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_zlim(0, lim * 1.2 + b_height)

        self.ax.set_xlabel("X Axis (mm)", fontweight="bold")
        self.ax.set_ylabel("Y Axis (mm)", fontweight="bold")
        self.ax.set_zlabel("Z Axis - Height (mm)", fontweight="bold")
        self.ax.set_title("3D Solid Industrial Robot Arm Simulator", fontsize=11, fontweight="bold")

        # Set Camera Angles from Sliders
        self.ax.view_init(elev=self.cam_elev, azim=self.cam_azim)

        ee_real = points[-1] + pedestal_top
        self.lbl_ee_x.config(text=f"End-Effector X: {ee_real[0]:8.2f} mm")
        self.lbl_ee_y.config(text=f"End-Effector Y: {ee_real[1]:8.2f} mm")
        self.lbl_ee_z.config(text=f"End-Effector Z: {ee_real[2]:8.2f} mm (Height from Ground)")

        self.canvas.draw_idle()

    def on_azim_slider_change(self, val):
        self.cam_azim = float(val)
        self.azim_ent.delete(0, tk.END)
        self.azim_ent.insert(0, f"{self.cam_azim:.0f}")
        self.update_simulation()

    def on_azim_entry_change(self, event):
        try:
            val = float(self.azim_ent.get())
            self.cam_azim = val
            self.azim_sld.set(val)
            self.update_simulation()
        except ValueError:
            pass

    def on_elev_slider_change(self, val):
        self.cam_elev = float(val)
        self.elev_ent.delete(0, tk.END)
        self.elev_ent.insert(0, f"{self.cam_elev:.0f}")
        self.update_simulation()

    def on_elev_entry_change(self, event):
        try:
            val = float(self.elev_ent.get())
            self.cam_elev = val
            self.elev_sld.set(val)
            self.update_simulation()
        except ValueError:
            pass

    def set_current_as_zero(self, idx):
        self.joint_offsets[idx] += self.joint_values[idx]
        self.joint_values[idx] = 0.0

        if idx < len(self.val_sliders):
            self.val_sliders[idx].set(0.0)
        if idx < len(self.val_entries):
            self.val_entries[idx].delete(0, tk.END)
            self.val_entries[idx].insert(0, "0.0")

        self.update_simulation()

    def on_dof_change(self):
        self.n_joints = self.dof_var.get()
        self.rebuild_joint_inputs()
        self.update_simulation()

    def on_base_slider_change(self, val):
        self.base_height = float(val)
        self.base_ent.delete(0, tk.END)
        self.base_ent.insert(0, f"{self.base_height:.0f}")
        self.update_simulation()

    def on_base_entry_change(self, event):
        try:
            val = float(self.base_ent.get())
            self.base_height = val
            self.base_sld.set(val)
            self.update_simulation()
        except ValueError:
            pass

    def on_thick_slider_change(self, val):
        self.link_radius = float(val)
        self.thick_ent.delete(0, tk.END)
        self.thick_ent.insert(0, f"{self.link_radius:.0f}")
        self.update_simulation()

    def on_thick_entry_change(self, event):
        try:
            val = float(self.thick_ent.get())
            self.link_radius = val
            self.thick_sld.set(val)
            self.update_simulation()
        except ValueError:
            pass

    def on_mode_change(self, idx, combo_val):
        if "Horizontal" in combo_val:
            self.joint_modes[idx] = "R_YAW"
        elif "Vertical" in combo_val:
            self.joint_modes[idx] = "R_PITCH"
        else:
            self.joint_modes[idx] = "P"

        self.joint_values[idx] = 30.0 if "Revolute" in combo_val else 0.0
        self.joint_offsets[idx] = 0.0
        self.rebuild_joint_inputs()
        self.update_simulation()

    def on_slider_change(self, idx, val):
        val_flt = float(val)
        self.joint_values[idx] = val_flt
        if idx < len(self.val_entries):
            self.val_entries[idx].delete(0, tk.END)
            self.val_entries[idx].insert(0, f"{val_flt:.1f}")
        self.update_simulation()

    def on_val_entry_change(self, idx, text_val):
        try:
            val = float(text_val)
            self.joint_values[idx] = val
            if idx < len(self.val_sliders):
                self.val_sliders[idx].set(val)
            self.update_simulation()
        except ValueError:
            pass

    def on_len_entry_change(self, idx, text_val):
        try:
            val = float(text_val)
            self.link_lengths[idx] = val
            self.update_simulation()
        except ValueError:
            pass


if __name__ == "__main__":
    root = tk.Tk()
    app = IndustrialRobotGUI(root)
    root.mainloop()
