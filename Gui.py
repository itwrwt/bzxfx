# -*- coding: utf-8 -*-
"""无人车控制台 UI（树莓派端）

布局参考电脑端控制台 ui.py：
  - 左侧: 主显示区（激光雷达扫描画布，黑底）+ 状态栏 + 智能体信息栏
  - 右侧: 固定 300px 控制列（速度档位 / 彩色九宫格 / 任务指令 / 日志）

手动控制支持键盘: W 前进 / X 后退 / Q 左移 / E 右移 / A 加速 / D 减速 /
S 或 空格 停止，方向键同 W/X/Q/E。
"""
import tkinter as tk
from tkinter import filedialog, ttk
import tkinter.font as tkfont
import math
import os
import subprocess
import sys
import threading
import time
try:
    from PIL import Image, ImageTk
except ImportError:
    Image = ImageTk = None

from constants import speed
from action_package import *
from qwen2AI import call_with_messages
from action_sequence_package import *
from radar_bridge import RadarBridge

# ---------- 布局常量（与电脑端 ui.py 保持一致的视觉比例） ----------
MAIN_CANVAS_WIDTH = 760
MAIN_CANVAS_HEIGHT = 570
CONTROL_PANEL_WIDTH = 300

# 方向键松开后是否自动发停止指令（保持型控制；改为 False 则沿用点按式）
STOP_ON_KEY_RELEASE = True

# ---------- 中文叠加 ----------
CJK_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "WenQuanYi Zen Hei",
    "WenQuanYi Micro Hei",
    "Source Han Sans SC",
    "Droid Sans Fallback",
    "AR PL UMing CN",
    "Microsoft YaHei",
    "SimHei",
    "DejaVu Sans",
)


def install_chinese_fonts():
    """把 Tk 的默认字体换成支持中文的字体，避免界面出现方块乱码。

    返回实际选中的字体名；没有任何可用中文字体时返回 None。
    """
    try:
        families = set(tkfont.families())
    except Exception:
        return None
    chosen = next((name for name in CJK_FONT_CANDIDATES if name in families), None)
    if chosen is None:
        return None
    for font_name in ("TkDefaultFont", "TkTextFont", "TkFixedFont", "TkMenuFont",
                      "TkHeadingFont", "TkCaptionFont", "TkSmallCaptionFont",
                      "TkIconFont", "TkTooltipFont"):
        try:
            tkfont.nametofont(font_name).configure(family=chosen)
        except Exception:
            pass
    return chosen


class UnmannedVehicleUI:
    def __init__(self, root):
        self.root = root
        self.root.title("小车控制台 (树莓派端)")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.cjk_font = install_chinese_fonts()
        self.font_family = self.cjk_font or ""

        _sw = root.winfo_screenwidth()
        _sh = root.winfo_screenheight()
        _h = min(680, max(520, _sh - 50))
        _w = min(1140, max(900, _sw - 30))
        root.geometry("%dx%d+%d+%d" % (
            _w, _h,
            max(0, (_sw - _w) // 2),
            max(0, (_sh - _h) // 2 - 30),
        ))
        root.resizable(True, True)

        self.speed = speed
        self.radar = RadarBridge()
        self.radar_on = False
        self.running = True
        self.status = "启动完成"
        self._keys_held = set()
        self._log_lines = ["程序已启动"]
        self.log_window = None
        self.log_text = None
        self.mapping_process = None

        # 先布局右侧控制列，保证小屏幕下控制按钮也不会被挤出窗口
        self._build_right_panel()
        self._build_left_panel()
        self._bind_keyboard()

        self.update_radar_view()
        self.root.after(150, self.update_status_bar)

    # ------------------------------------------------------------------
    # 左侧: 主显示区
    # ------------------------------------------------------------------
    def _build_left_panel(self):
        left = ttk.Frame(self.root)
        left.pack(side="left", fill="both", expand=True, padx=(6, 4), pady=6)
        video_host = ttk.Frame(left)
        video_host.pack(fill="both", expand=True)
        video_host.pack_propagate(False)
        video_host.columnconfigure(0, weight=1)
        video_host.rowconfigure(0, weight=1)

        self.radar_canvas = tk.Canvas(
            video_host, bg="#101418", highlightthickness=0,
            width=MAIN_CANVAS_WIDTH, height=MAIN_CANVAS_HEIGHT,
            scrollregion=(0, 0, MAIN_CANVAS_WIDTH, MAIN_CANVAS_HEIGHT),
        )
        self.radar_canvas.grid(row=0, column=0, sticky="nsew")
        x_scroll = ttk.Scrollbar(video_host, orient="horizontal",
                                 command=self.radar_canvas.xview)
        y_scroll = ttk.Scrollbar(video_host, orient="vertical",
                                 command=self.radar_canvas.yview)
        x_scroll.grid(row=1, column=0, sticky="ew")
        y_scroll.grid(row=0, column=1, sticky="ns")
        self.radar_canvas.configure(xscrollcommand=x_scroll.set,
                                    yscrollcommand=y_scroll.set)

        # 状态栏（同电脑端样式: 浅灰细字）
        self.lbl_status = tk.Label(left, text="", justify="left", anchor="w",
                                   font=(self.font_family, 10), fg="#333",
                                   wraplength=MAIN_CANVAS_WIDTH)
        self.lbl_status.pack(fill="x", pady=(4, 0))

        # 智能体信息栏（同电脑端样式: 深底浅字）
        self.lbl_agent = tk.Label(left, text="智能体输入:", justify="left",
                                  anchor="w", bg="#111", fg="#d0d0d0",
                                  font=(self.font_family, 9),
                                  wraplength=MAIN_CANVAS_WIDTH)
        self.lbl_agent.pack(fill="x", pady=(4, 0))

    # ------------------------------------------------------------------
    # 右侧: 控制列
    # ------------------------------------------------------------------
    def _build_right_panel(self):
        right = ttk.Frame(self.root, width=CONTROL_PANEL_WIDTH)
        right.pack(side="right", fill="y", anchor="n", padx=(4, 6), pady=6)
        right.pack_propagate(False)

        ttk.Label(right, text="小车控制",
                  font=(self.font_family, 13, "bold")).pack(pady=(1, 2))

        # ---- 速度档位 ----
        ttk.Label(right, text="速度档位 (加速a / 减速d):").pack(anchor="w")
        speed_options = [f"{key}: {value}" for key, value in self.speed.items()]
        self.speed_var = tk.StringVar(value=speed_options[0])
        speed_menu = ttk.Combobox(right, textvariable=self.speed_var,
                                  values=speed_options, state="readonly")
        speed_menu.pack(fill="x", pady=(1, 2))
        speed_menu.bind("<<ComboboxSelected>>", self.update_speed)

        # ---- 彩色九宫格 (3x3) ----
        ttk.Label(right, text="手动控制:").pack(anchor="w")
        grid = ttk.Frame(right)
        grid.pack(pady=1)
        layout = [
            ("左旋", self.cmd_rotate_left, "#2196f3", 0, 0),
            ("前进", self.cmd_forward, "#4caf50", 0, 1),
            ("右旋", self.cmd_rotate_right, "#2196f3", 0, 2),
            ("左移", self.cmd_left, "#2196f3", 1, 0),
            ("停止", self.cmd_stop, "#f44336", 1, 1),
            ("右移", self.cmd_right, "#2196f3", 1, 2),
            ("加速", self.cmd_accelerate, "#607d8b", 2, 0),
            ("后退", self.cmd_backward, "#ff9800", 2, 1),
            ("减速", self.cmd_decelerate, "#607d8b", 2, 2),
        ]
        for text, cmd, color, r, c in layout:
            b = tk.Button(grid, text=text, command=cmd,
                          bg=color, fg="white",
                          activebackground=color,
                          font=(self.font_family, 9),
                          height=1, width=5, bd=0)
            b.grid(row=r, column=c, padx=2, pady=1, ipadx=1, ipady=1)

        utility_row = ttk.Frame(right)
        utility_row.pack(fill="x", pady=(0, 1))
        tk.Button(utility_row, text="清空输入", command=self.reset_action,
                  bg="#7b1fa2", fg="white", bd=0,
                  font=(self.font_family, 9), height=1).pack(
                      side="left", expand=True, fill="x", padx=(0, 2))
        tk.Button(utility_row, text="运行日志", command=self.open_log_window,
                  bg="#607d8b", fg="white", bd=0,
                  font=(self.font_family, 9), height=1).pack(
                      side="left", expand=True, fill="x", padx=(2, 0))

        # ---- 激光雷达（独立区块，状态一目了然） ----
        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=(4, 3))
        ttk.Label(right, text="激光雷达").pack(anchor="w")
        radar_row = ttk.Frame(right)
        radar_row.pack(fill="x", pady=2)
        self.btn_radar_start = tk.Button(radar_row, text="启动雷达",
                                         command=self.start_radar,
                                         bg="#2e7d32", fg="white", bd=0,
                                         font=(self.font_family, 10),
                                         width=8, height=1)
        self.btn_radar_start.pack(side="left", padx=(0, 4))
        self.btn_radar_stop = tk.Button(radar_row, text="停止雷达",
                                        command=self.stop_radar,
                                        bg="#9e9e9e", fg="white", bd=0,
                                        font=(self.font_family, 10),
                                        width=8, height=1)
        self.btn_radar_stop.pack(side="left", padx=(4, 0))
        self.radar_status = tk.Label(right, text="状态: 未启动",
                                     anchor="w", justify="left",
                                     bg="#111", fg="#b9f6ca",
                                     font=(self.font_family, 9))
        self.radar_status.pack(fill="x", pady=(1, 0))

        # ---- 任务指令 ----
        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=(4, 3))
        ttk.Label(right, text="任务指令 (大模型):").pack(anchor="w")
        self.input_text = tk.Text(right, height=2,
                                  font=(self.font_family, 9))
        self.input_text.pack(fill="x", pady=(1, 2))
        task_row = ttk.Frame(right)
        task_row.pack(pady=(0, 1))
        ttk.Button(task_row, text="规划并执行任务", command=self.confirm_action,
                   width=15).pack(side="left", padx=(0, 4))
        ttk.Button(task_row, text="急停", command=self.cmd_stop,
                   width=9).pack(side="left", padx=(4, 0))

        # ---- 自主建图 ----
        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=(4, 3))
        ttk.Label(right, text="自主建图").pack(anchor="w")
        mapping_row = ttk.Frame(right)
        mapping_row.pack(fill="x", pady=1)
        self.mapping_manual_btn = ttk.Button(
            mapping_row, text="手动建图", command=self.start_manual_mapping
        )
        self.mapping_manual_btn.pack(side="left", expand=True, fill="x", padx=1)
        self.mapping_auto_btn = ttk.Button(
            mapping_row, text="自动建图", command=self.start_auto_mapping
        )
        self.mapping_auto_btn.pack(side="left", expand=True, fill="x", padx=1)
        self.mapping_stop_btn = ttk.Button(
            mapping_row, text="强制停止", command=self.stop_mapping
        )
        self.mapping_stop_btn.pack(side="left", expand=True, fill="x", padx=1)
        mapping_files_row = ttk.Frame(right)
        mapping_files_row.pack(fill="x", pady=(0, 1))
        self.mapping_save_btn = ttk.Button(
            mapping_files_row, text="保存地图", command=self.save_mapping
        )
        self.mapping_save_btn.pack(side="left", expand=True, fill="x", padx=1)
        self.mapping_view_btn = ttk.Button(
            mapping_files_row, text="打开地图", command=self.open_saved_map
        )
        self.mapping_view_btn.pack(side="left", expand=True, fill="x", padx=1)
        self.mapping_live_btn = ttk.Button(
            mapping_files_row, text="实时地图", command=self.open_live_map
        )
        self.mapping_live_btn.pack(side="left", expand=True, fill="x", padx=1)
        self.mapping_status = tk.Label(right, text="状态: 未运行", anchor="w",
                                       justify="left", fg="#555",
                                       font=(self.font_family, 9))
        self.mapping_status.pack(fill="x", pady=(2, 0))

        # ---- 底部 ----
        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=(4, 2))
        tk.Button(right, text="退出程序", command=self.on_close,
                  bg="#f44336", fg="white", bd=0,
                  font=(self.font_family, 9), height=1).pack(fill="x", pady=1)

    # ------------------------------------------------------------------
    # 键盘控制（同电脑端: 按下生效, 松开停止）
    # ------------------------------------------------------------------
    def _bind_keyboard(self):
        for k in ("w", "x", "q", "e", "a", "d", "s"):
            self.root.bind("<KeyPress-%s>" % k,
                           lambda e, kk=k: self._key_press(kk))
            self.root.bind("<KeyRelease-%s>" % k,
                           lambda e, kk=k: self._key_release(kk))
        self.root.bind("<KeyPress-Up>", lambda e: self._key_press("w"))
        self.root.bind("<KeyRelease-Up>", lambda e: self._key_release("w"))
        self.root.bind("<KeyPress-Down>", lambda e: self._key_press("x"))
        self.root.bind("<KeyRelease-Down>", lambda e: self._key_release("x"))
        self.root.bind("<KeyPress-Left>", lambda e: self._key_press("q"))
        self.root.bind("<KeyRelease-Left>", lambda e: self._key_release("q"))
        self.root.bind("<KeyPress-Right>", lambda e: self._key_press("e"))
        self.root.bind("<KeyRelease-Right>", lambda e: self._key_release("e"))
        self.root.bind("<KeyPress-space>", lambda e: self.cmd_stop())
        self.root.bind("<Escape>", lambda e: self.cmd_stop())

    def _typing_in_input(self):
        return self.root.focus_get() == self.input_text

    def _key_press(self, key):
        if self._typing_in_input():
            return
        if key in self._keys_held:
            return
        self._keys_held.add(key)
        handler = {
            "w": self.cmd_forward, "x": self.cmd_backward,
            "q": self.cmd_left, "e": self.cmd_right,
            "a": self.cmd_accelerate, "d": self.cmd_decelerate,
            "s": self.cmd_stop,
        }.get(key)
        if handler:
            handler()

    def _key_release(self, key):
        self._keys_held.discard(key)
        if (STOP_ON_KEY_RELEASE and not self._keys_held
                and key in ("w", "x", "q", "e")):
            # 保持型方向键: 松开即停（速度档位 a/d 是脉冲，不在此列）
            try:
                suspend()
            except Exception:
                pass
            self.status = "松开 %s -> 停止" % key
            self._log("键盘松开 %s，已停止" % key)

    # ------------------------------------------------------------------
    # 手动指令
    # ------------------------------------------------------------------
    def cmd_forward(self):
        self._manual("前进", move_forward)

    def cmd_backward(self):
        self._manual("后退", backward)

    def cmd_left(self):
        self._manual("左移", left)

    def cmd_right(self):
        self._manual("右移", right)

    def cmd_rotate_left(self):
        self._manual("左旋", lambda: send_serial_data(bytes([0x55, 0x55, 0x03, 0x0D, 0x80])))

    def cmd_rotate_right(self):
        self._manual("右旋", lambda: send_serial_data(bytes([0x55, 0x55, 0x03, 0x0E, 0x80])))

    def cmd_accelerate(self):
        self._manual("加速", accelerate, pulse=True)

    def cmd_decelerate(self):
        self._manual("减速", decelerate, pulse=True)

    def cmd_stop(self):
        self._manual("停止", suspend)

    def _manual(self, name, func, pulse=False):
        try:
            func()
        except Exception as exc:
            self.status = "%s 失败: %s" % (name, exc)
            self._log("%s 执行失败: %s" % (name, exc))
            return
        self.status = "指令: %s%s" % (name, " (脉冲)" if pulse else "")
        self._log("手动指令: %s" % name)

    # ------------------------------------------------------------------
    # 激光雷达
    # ------------------------------------------------------------------
    def start_radar(self):
        ok, message = self.radar.start()
        self.radar_on = ok
        self.status = "激光雷达: %s" % message
        self._log("激光雷达: %s" % message)
        self._refresh_radar_controls()

    def _mapping_script(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "lidar_mapping.py")

    def _run_mapping_command(self, command, name=None):
        args = [sys.executable, self._mapping_script(), command]
        if name:
            args.append(name)
        return subprocess.run(args, capture_output=True, text=True, timeout=60)

    def _run_mapping_command_async(self, command, callback, name=None):
        def worker():
            try:
                result = self._run_mapping_command(command, name)
                error = None
            except Exception as exc:
                result = None
                error = str(exc)
            self.root.after(0, lambda: callback(result, error))

        threading.Thread(target=worker, daemon=True).start()

    def start_manual_mapping(self):
        try:
            suspend()
        except Exception:
            pass
        self.start_mapping("manual")

    def start_auto_mapping(self):
        self.start_mapping("auto")

    def start_mapping(self, mode="manual"):
        self.mapping_status.config(text="建图启动中…")
        self._log("正在请求启动%s建图" % ("自动" if mode == "auto" else "手动"))
        self._run_mapping_command_async(
            mode,
            lambda result, error: self._mapping_command_finished(
                result, error, open_live=(mode == "auto")
            ),
        )

    def stop_mapping(self):
        self.mapping_status.config(text="正在强制停止…")
        self._log("正在强制停止建图（不保存地图）")
        self._run_mapping_command_async("force-stop", self._mapping_command_finished)

    def _mapping_command_finished(self, result, error, open_live=False):
        if error:
            message = "建图操作失败: %s" % error
        else:
            output = (result.stdout or result.stderr or "建图命令无输出").strip()
            lines = [line for line in output.splitlines() if line.strip()]
            message = "；".join(lines[-2:]) if lines else "建图命令无输出"
            if result.returncode:
                message = "退出码 %s；%s" % (result.returncode, message)
        self.mapping_status.config(text=message[-120:])
        self.status = message
        self._log(message)
        if open_live and not error and result is not None and result.returncode == 0:
            self._run_mapping_command_async("view", self._live_view_command_finished)

    def open_live_map(self):
        self._run_mapping_command_async("view", self._live_view_command_finished)

    def _live_view_command_finished(self, result, error):
        if error:
            message = "实时地图窗口启动失败: %s" % error
        else:
            output = (result.stdout or result.stderr or "实时地图窗口已打开").strip()
            message = "；".join(line.strip() for line in output.splitlines() if line.strip())
            if result.returncode:
                message = "退出码 %s；%s" % (result.returncode, message)
        self.mapping_status.config(text=message[-120:])
        self._log(message)

    def save_mapping(self):
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        name = "map_%s_%03d" % (timestamp, int((time.time() % 1) * 1000))
        self.mapping_status.config(text="正在保存地图…")
        self._log("地图保存请求: %s" % name)
        self._run_mapping_command_async("stop", self._mapping_command_finished, name)

    def open_saved_map(self):
        map_dir = os.path.expanduser("~/car/maps")
        selected_path = filedialog.askopenfilename(
            parent=self.root,
            title="选择要打开的地图",
            initialdir=map_dir if os.path.isdir(map_dir) else os.path.expanduser("~"),
            filetypes=(
                ("地图文件 (YAML / PGM)", ("*.yaml", "*.yml", "*.pgm")),
                ("地图元数据 (YAML)", ("*.yaml", "*.yml")),
                ("地图图像 (PGM)", "*.pgm"),
            ),
        )
        if not selected_path:
            return

        selected_path = os.path.abspath(selected_path)
        extension = os.path.splitext(selected_path)[1].lower()
        yaml_path = selected_path if extension in (".yaml", ".yml") else os.path.splitext(selected_path)[0] + ".yaml"
        image_path = selected_path if extension == ".pgm" else None
        try:
            if image_path is None and os.path.isfile(yaml_path):
                with open(yaml_path, "r", encoding="utf-8") as stream:
                    for line in stream:
                        if line.strip().startswith("image:"):
                            image_name = line.split(":", 1)[1].strip().strip("\\\"'")
                            image_path = image_name if os.path.isabs(image_name) else os.path.join(
                                os.path.dirname(yaml_path), image_name
                            )
                            break
            if image_path is None:
                image_path = os.path.splitext(yaml_path)[0] + ".pgm"
            if not os.path.isfile(image_path):
                raise FileNotFoundError("未找到地图图像: " + image_path)
            window = tk.Toplevel(self.root)
            window.title("已完成地图 - " + os.path.basename(image_path))
            window.geometry("900x680")
            holder = ttk.Frame(window)
            holder.pack(fill="both", expand=True)
            canvas = tk.Canvas(holder, bg="#202428", highlightthickness=0)
            xbar = ttk.Scrollbar(holder, orient="horizontal", command=canvas.xview)
            ybar = ttk.Scrollbar(holder, orient="vertical", command=canvas.yview)
            canvas.configure(xscrollcommand=xbar.set, yscrollcommand=ybar.set)
            canvas.grid(row=0, column=0, sticky="nsew")
            ybar.grid(row=0, column=1, sticky="ns")
            xbar.grid(row=1, column=0, sticky="ew")
            holder.rowconfigure(0, weight=1)
            holder.columnconfigure(0, weight=1)
            bitmap = None
            if Image is not None and ImageTk is not None:
                bitmap = Image.open(image_path).convert("RGB")
                photo = ImageTk.PhotoImage(bitmap)
            else:
                photo = tk.PhotoImage(file=image_path)
            canvas.create_image(0, 0, image=photo, anchor="nw")
            canvas.configure(scrollregion=(0, 0, photo.width(), photo.height()))
            window.map_photo = photo
            window.map_bitmap = bitmap
            if os.path.isfile(yaml_path):
                with open(yaml_path, "r", encoding="utf-8") as stream:
                    meta = [line.strip() for line in stream if line.startswith(("resolution:", "origin:"))]
                ttk.Label(window, text="  ".join(meta), anchor="w").pack(fill="x")
            self._log("已打开地图: %s" % image_path)
        except Exception as exc:
            self.status = "地图打开失败: %s" % exc
            self._log(self.status)

    def stop_radar(self):
        self.radar.stop()
        self.radar_on = False
        self.status = "激光雷达已停止"
        self._log("激光雷达已停止")
        self._refresh_radar_controls()

    def _refresh_radar_controls(self):
        """启动/停止按钮互斥高亮 + 状态文字，一眼能看出雷达开没开。"""
        if self.radar_on:
            self.btn_radar_start.config(text="运行中", bg="#9e9e9e")
            self.btn_radar_stop.config(text="停止雷达", bg="#f44336")
            self.radar_status.config(text="状态: 运行中（左侧画布实时刷新）",
                                     fg="#b9f6ca")
        else:
            self.btn_radar_start.config(text="启动雷达", bg="#2e7d32")
            self.btn_radar_stop.config(text="停止雷达", bg="#9e9e9e")
            self.radar_status.config(text="状态: 未启动", fg="#b0bec5")

    def update_radar_view(self):
        if not self.running:
            return
        points, stamp, status = self.radar.snapshot()
        canvas = self.radar_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), MAIN_CANVAS_WIDTH)
        height = max(canvas.winfo_height(), MAIN_CANVAS_HEIGHT)
        cx, cy = width / 2, height / 2
        radius = min(width, height) * 0.43
        max_range = 8.0
        for fraction in (0.25, 0.5, 0.75, 1.0):
            r = radius * fraction
            canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                               outline="#2d4655")
            canvas.create_text(cx + 5, cy - r - 8,
                               text="%.1fm" % (max_range * fraction),
                               fill="#7f9aa8", anchor="w")
        canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#2d4655")
        canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#2d4655")
        for angle, distance in points:
            d = min(distance, max_range) / max_range * radius
            x = cx + math.sin(angle) * d
            y = cy - math.cos(angle) * d
            canvas.create_oval(x - 2, y - 2, x + 2, y + 2,
                               fill="#48e0a4", outline="")
        age = time.time() - stamp if stamp else None
        suffix = " | %.1fs前" % age if age is not None else ""
        hint = "%s%s | 点数: %d" % (status, suffix, len(points))
        canvas.create_text(10, 10, anchor="nw", text=hint,
                           fill="#b9f6ca", font=(self.font_family, 11))
        self.root.after(100, self.update_radar_view)

    # ------------------------------------------------------------------
    # 状态栏 / 日志
    # ------------------------------------------------------------------
    def _log(self, message):
        stamp = time.strftime("%H:%M:%S")
        self._log_lines.append("[%s] %s" % (stamp, message))
        self._log_lines = self._log_lines[-400:]
        print(message)
        if self.log_text is not None:
            try:
                self.log_text.config(state="normal")
                self.log_text.insert(tk.END, "[%s] %s\n" % (stamp, message))
                self.log_text.see(tk.END)
                self.log_text.config(state="disabled")
            except tk.TclError:
                self.log_text = None

    def update_status_bar(self):
        if not self.running:
            return
        _points, _stamp, radar_status = self.radar.snapshot()
        self.lbl_status.config(text="雷达: %s | 模式: 手动\n%s"
                               % (radar_status, self.status))
        self.lbl_agent.config(
            text="智能体输入:\n在右侧输入中文任务，点“规划并执行任务”")
        self.root.after(150, self.update_status_bar)

    def open_log_window(self):
        if self.log_window is not None and self.log_window.winfo_exists():
            self.log_window.lift()
            return
        self.log_window = tk.Toplevel(self.root)
        self.log_window.title("运行日志")
        self.log_window.geometry("640x420")
        self.log_text = tk.Text(self.log_window, wrap="word", state="disabled",
                                bg="#111", fg="#d0d0d0",
                                font=(self.font_family, 10))
        self.log_text.pack(fill="both", expand=True)
        self.log_text.config(state="normal")
        self.log_text.insert(tk.END, "\n".join(self._log_lines) + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state="disabled")

    # ------------------------------------------------------------------
    # 大模型任务
    # ------------------------------------------------------------------
    def confirm_action(self):
        threading.Thread(target=self.process_confirm_action,
                         daemon=True).start()

    def process_confirm_action(self):
        self.root.after(0, self.show_progress_dialog)
        try:
            message = self.big_model()
        except Exception as exc:
            message = "任务执行失败: %s" % exc
        self.root.after(0, lambda: self._after_task(message))
        self.root.after(0, self.hide_progress_dialog)

    def _after_task(self, message):
        self._log("任务结果: %s" % message)
        self.status = "任务完成: %s" % str(message)[:80]

    def big_model(self):
        input_content = self.input_text.get("1.0", tk.END).strip()
        if not input_content:
            return "任务指令输入框为空，请输入指令。"
        current_speed_text = self.speed_var.get()
        current_speed_key, current_speed_value = current_speed_text.split(": ")
        current_speed_value = float(current_speed_value)
        prompt = (
            f"假如你是台智能小车，已知 你的速度{current_speed_value}cm/s,"
            f"{input_content},我希望你给我输出像下面例子的信息"
            "{'action': 'w', //'w'表示前进 'q'表示左移 'e'表示右移 "
            "'s'表示停止,'distance': //表示距离，以厘米为单位},"
            "请不要输出推理过程，直接给出json数组即可"
        )
        print(prompt)
        json_data = call_with_messages(prompt)
        action_list = list_to_Actionlist(json_data, current_speed_value)
        action_to_move(action_list)
        if not json_data:
            return ("大模型处理失败，返回信息: 任务失败\n输入的指令是: %s\n"
                    "当前速度是: %s, 值为: %s"
                    % (input_content, current_speed_key, current_speed_value))
        return "%s" % json_data

    def show_progress_dialog(self):
        self.progress_dialog = tk.Toplevel(self.root)
        self.progress_dialog.title("处理中...")
        w, h = 300, 100
        sw = self.progress_dialog.winfo_screenwidth()
        sh = self.progress_dialog.winfo_screenheight()
        self.progress_dialog.geometry(
            "%dx%d+%d+%d" % (w, h, (sw - w) // 2, (sh - h) // 2))
        ttk.Label(self.progress_dialog,
                  text="车辆正在运动中，请稍候...").pack(padx=20, pady=20)

    def hide_progress_dialog(self):
        dialog = getattr(self, "progress_dialog", None)
        if dialog is not None and dialog.winfo_exists():
            dialog.destroy()

    # ------------------------------------------------------------------
    # 其它
    # ------------------------------------------------------------------
    def update_speed(self, event=None):
        selected = self.speed_var.get()
        self.status = "速度更新为: %s" % selected
        self._log("速度更新为: %s" % selected)

    def reset_action(self):
        self.input_text.delete("1.0", tk.END)
        self.status = "已清空任务指令"
        self._log("已清空任务指令")

    def on_close(self):
        self.running = False
        try:
            self._run_mapping_command("close-viewer")
        except Exception:
            pass
        try:
            self._run_mapping_command("force-stop")
        except Exception:
            pass
        try:
            self.radar.close()
        except Exception:
            pass
        self.root.destroy()


# 运行主程序
if __name__ == "__main__":
    root = tk.Tk()
    app = UnmannedVehicleUI(root)
    root.mainloop()
