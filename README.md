# Robotic Navigation Final | 機器人導航系統

**課程**：機器導航與探索（Robotic Navigation and Exploration）

---

## 📋 專案介紹 — Description

本專案是一個完整的**移動機器人自主導航系統**，整合了**SLAM定位建圖**、**路徑規劃導航**、**視覺目標檢測**和**即時避障**等多個模組。

系統架構基於 **ROS 2 + Python**，支援實際機器人硬體與虛擬 Unity 環境並行運行。主要功能包括：
- 🗺️ 地圖掃描與建圖（SLAM）
- 📍 定位與導航（Nav2 框架）
- 🎯 YOLOv11 物體檢測與追蹤
- ⚡ LiDAR + 深度相機雙重避障
- 🧭 自動探索與目標導航

---

## 🏗️ 系統架構圖 — System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Robot Control System                      │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌────────────────┐          ┌──────────────────────┐       │
│  │  pros_car      │◄────────►│  pros_app (Docker)   │       │
│  │ (On Robot)     │          │  SLAM / Nav2 / AMCL  │       │
│  │                │          │                      │       │
│  │ • Motor Ctrl   │          │ • Map Build & Store  │       │
│  │ • Sensors Pub  │          │ • Path Planning      │       │
│  │ • Arm Control  │          │ • Localization       │       │
│  └────────────────┘          └──────────────────────┘       │
│         ▲                              ▲                     │
│         │                              │                     │
│    ROS2 Topics                   ROS2 Bridge Network         │
│         │                              │                     │
│         └──────────┬─────────────┬─────┘                     │
│                    │             │                           │
│         ┌──────────▼───────┐ ┌──▼─────────────────┐          │
│         │  YOLOv11 Pkg      │ │  Foxglove Studio   │          │
│         │  (Detection &    │ │  (Visualization)   │          │
│         │   Segmentation)  │ │                    │          │
│         └──────────────────┘ └────────────────────┘          │
│                                                               │
│  ┌──────────────────────────────────────────────┐           │
│  │   Motor Commands / Arm Commands / Goal Pose  │           │
│  └──────────────────────────────────────────────┘           │
│                     ▼                                        │
│         ┌───────────────────────┐                           │
│         │  Real Robot / Unity   │                           │
│         │  Simulation           │                           │
│         └───────────────────────┘                           │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 📦 這個 Repo 包含的內容

### 1. **`pros/`** — 機器人主控制系統

#### `pros_car/` — 車體控制（搭載在實體機器人上）
- **功能**：
  - 手動車體控制（前進、後退、轉向、旋轉）
  - 機械臂控制（5軸關節角度調整）
  - 自動導航模式整合
  - 傳感器數據發佈（里程計、IMU、相機）
  
- **核心模組**：
  - `car_controller.py` — **主控制邏輯**
    - 管理導航線程與停止事件
    - 支援三種導航模式：`manual_auto_nav`、`target_auto_nav`、`custom_nav`
    - 當偵測到皮卡丘時調用 `nav2_target()` 和 `camera_nav_unity()`
  
  - `nav_processing.py` — **導航處理器**
    - 路徑規劃與動作計算
    - **自動探索邏輯**：記錄 `self.visited_map` 追蹤未探索區域
    - 避障與目標追蹤函數
    - Frontier-based 探索算法
  
  - `robot_control` — 使用者介面入口點

#### `pros_app/` — 機器人應用層（Docker 容器化）
- **功能**：
  - SLAM 建圖與地圖管理
  - Nav2 路徑規劃框架
  - AMCL 定位與地圖儲存
  - 多相機支援（Astra / Dabai 深度相機）
  
- **特色**：
  - 自定義 Docker Bridge Network（減少網路延遲）
  - ROS2 圖像壓縮（從 210 Mbps 壓至 24-32 Mbps）
  - Foxglove 可視化整合

---

### 2. **`ros2_yolo_integration/`** — YOLO 物體檢測系統

#### `yolo_pkg/` — YOLO 檢測節點
- **模式**：
  - Mode 1：邊界框標記
  - Mode 2：邊界框 + 截圖
  - Mode 3：5fps 相機截圖
  - Mode 4：**語義分割模式**（重點）

- **核心函數**：
  - `draw_bounding_boxes()` — 繪製檢測結果
  - `save_fps_screenshot()` — 固定速率截圖

#### `yolo_example_pkg/` — 檢測示例包

- **模型配置**：
  - 放置 `.pt` 模型檔案於 `models/` 目錄
  - 支援 `yolov11m.pt`（檢測）與 `yolov11n-seg.pt`（分割）

---

## 🧠 Function Map — 功能地圖

```
CarController (car_controller.py)
├── manual_control(key)              ──► 手動控制
├── auto_control(mode, target, key)  ──► 自動導航
│   ├── manual_auto_nav              ──► 接收 Foxglove 座標導航
│   ├── target_auto_nav              ──► 預設目標點循環導航
│   └── custom_nav                   ──► 自動探索 + 目標追蹤
│       └── background_task()        ──► 後台導航線程
│
└─► NavProcessing (nav_processing.py)
    ├── exploration_logic()                          ──► 自動探索
    │   ├── find_closest_frontier_point()           ──► 找未知邊界
    │   └── mark_visited_point()                    ──► 標記已訪問區域
    │
    ├── get_action_from_nav2_plan_no_dynamic_p_2_p()  ──► 路徑跟蹤
    │   └── get_next_target_point()                  ──► 取下一路徑點
    │
    ├── nav2_target()                               ──► 深度相機目標導航
    │   └── calculate_angle_to_target()             ──► 計算方向角度
    │
    ├── camera_nav_unity()                          ──► 完整的視覺導航
    │   ├── 近距離避障（深度相機 & LiDAR）
    │   ├── 目標追蹤（YOLO 檢測）
    │   └── 牆邊探索（Wall Following）
    │
    └── filter_negative_onehundred()                ──► 深度資料清理

YOLO Detection Pipeline
├── yolo_detection_node                    ──► 主檢測節點
├── yolo_segmentation_model / detect_model ──► 模型選擇
└── draw_bounding_boxes()                  ──► 視覺化
```

---

## 🔑 核心實現細節

### 導航系統架構

**大方向的概念實現於 `pros_car/car_controller.py`**，主要邏輯流程如下：

1. **初始化**：建立導航線程管理機制
   - `_auto_nav_thread` — 後台導航執行緒
   - `_stop_event` — 線程停止事件
   - `_thread_running` — 線程狀態標記

2. **導航模式**：
   - **Manual Auto Nav**：接收外部 `/goal_pose` 座標
   - **Target Auto Nav**：循環遍歷預設目標列表
   - **Custom Nav**：整合自動探索 + 即時目標追蹤

3. **大局導航**（`nav_processing.py`）：
   - 使用額外的 `self.visited_map` 記錄未探索區域
   - 路徑規劃由內部已實現的函數負責
   - 與 Nav2 框架整合進行全局路徑計算

---

### 目標檢測與避障邏輯

**當偵測到皮卡丘時** — 實現在 `custom_nav` 模式中：

```python
if yolo_info and yolo_info[0] == 1 and target_label == "Pikachu":
    action_key_pikachu = self.nav_processing.nav2_target()
    # 或使用 self.nav_processing.camera_nav_unity()
```

#### 兩種追蹤函數說明：

| 函數 | 用途 | 特色 |
|------|------|------|
| **`nav2_target()`** | 基於深度相機的目標導航 | 利用 TF 座標轉換到地圖座標系 |
| **`camera_nav_unity()`** | 完整多傳感器融合導航 | LiDAR + 深度相機 + YOLO 組合避障 |

#### 模型重訓練背景

⚠️ **重要發現**：
- 原始 YOLO 模型的皮卡丘訓練集角度單一
- 在 **近距離（< 40cm）** 時，label 會消失導致追蹤失效
- 解決方案：使用 `yolo_pkg` 重新訓練物體偵測 & 分割模型
- 新模型已放入 `ros2_yolo_integration/` 供使用

---

## ⚙️ 技術棧

| 組件 | 技術 | 說明 |
|------|------|------|
| **底層框架** | ROS 2 (Humble) | 分佈式機器人中間件 |
| **主程式語言** | Python 3.x | 高效開發 |
| **導航算法** | Nav2 Stack | 成熟的路徑規劃框架 |
| **定位方案** | AMCL | 蒙特卡洛粒子濾波定位 |
| **視覺檢測** | YOLOv11 | 實時物體檢測與分割 |
| **距離傳感** | LiDAR + 深度相機 | 多層次避障 |
| **容器化** | Docker & Docker Compose | 環境一致性管理 |
| **可視化** | Foxglove Studio | 實時數據監控 |

---

## ⚠️ 注意事項 — Warnings

### 1. **YOLO 模型距離問題**
- 原始模型在目標距離 < 40cm 時無法正確標記
- 使用重訓練版本時務必注意模型路徑設置

### 2. **ROS 2 Topic 數據獲取**
- 從 ROS 2 topic 提取數據需要正確的座標系轉換
- `/yolo/detection/position` 需要 TF frame_id 才能轉換到地圖座標系
- 目前主要使用 `nav2_target()` 進行深度相機導航

### 3. **時間限制與未完成項目**
- 由於對 ROS 2 不夠熟悉，初期花費大量時間在數據獲取與 topic 訂閱
- **時間關係**，未能在環境內實現之前學到的控制演算法（如 PID 控制、RRT* 路徑規劃等）
- 當前避障邏輯為基礎狀態機實現，可作為未來優化方向

### 4. **多傳感器融合**
- LiDAR 與深度相機的數據融合仍有改進空間
- 建議檢查 `camera_nav_unity()` 中的避障閾值設定是否符合硬體特性

### 5. **Docker 網路配置**
- 使用自定義 bridge network 以降低延遲
- 若 `pros_app_my_bridge_network` 不存在會自動創建
- 首次啟動建議檢查網路連接

---

## 🚀 快速開始

### 啟動機器人系統
```bash
cd pros/pros_car
./car_control.sh
# 在容器內執行編譯與設置
r
# 啟動機器人控制界面
ros2 run pros_car_py robot_control
```

### 啟動應用層（SLAM / 定位）
```bash
cd pros/pros_app
./control.sh
# 依提示選擇模式（SLAM / 定位 / 地圖保存等）
```

### 啟動 YOLO 檢測
```bash
cd ros2_yolo_integration
./yolo_activate.sh
# 在容器內編譯
r
# 運行檢測節點
ros2 run yolo_pkg yolo_detection_node
```

---

## 📚 相關文件

- [pros_car README](./pros/pros_car/README.md) — 車體控制詳細文檔
- [pros_app README](./pros/pros_app/README.md) — 應用層詳細文檔  
- [ros2_yolo_integration README](./ros2_yolo_integration/README.md) — YOLO 集成指南

---

## 👥 貢獻者

- 陳麒麟
- 曾裕翔
- 林庭琮
- 鍾博丞

**指導教授**：胡敏君教授

---

## 📝 License

此專案為課程項目，詳見各子模組的授權聲明。

---

**最後更新**：2026-07-20  
**語言構成**：Python 91.8% | Shell 6.5% | Dockerfile 1.2% | Batchfile 0.5%
