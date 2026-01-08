# #!/home/airlab1/miniconda3/envs/ros1/bin/python3
# """
# 颜色形状检测节点
# 检测画面中的彩色几何图形，识别颜色、形状和3D位置
# """
# import rospy
# import pyrealsense2 as rs
# import numpy as np
# import cv2
# from vi_msgs.msg import ObjectInfo

# # ===================== 颜色定义 =====================
# COLOR_RANGES = {
#     'red': [
#         {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
#         {'lower': np.array([160, 100, 100]), 'upper': np.array([180, 255, 255])}
#     ],
#     'orange': [{'lower': np.array([11, 100, 100]), 'upper': np.array([25, 255, 255])}],
#     'yellow': [{'lower': np.array([26, 100, 100]), 'upper': np.array([34, 255, 255])}],
#     'green': [{'lower': np.array([35, 100, 100]), 'upper': np.array([85, 255, 255])}],
#     'cyan': [{'lower': np.array([86, 100, 100]), 'upper': np.array([100, 255, 255])}],
#     'blue': [{'lower': np.array([101, 100, 100]), 'upper': np.array([130, 255, 255])}],
#     'purple': [{'lower': np.array([131, 100, 100]), 'upper': np.array([159, 255, 255])}],
# }

# COLOR_BGR = {
#     'red': (0, 0, 255), 'orange': (0, 165, 255), 'yellow': (0, 255, 255),
#     'green': (0, 255, 0), 'cyan': (255, 255, 0), 'blue': (255, 0, 0), 'purple': (255, 0, 255),
# }


# def get_color_mask(hsv, color_name):
#     """获取指定颜色的掩码"""
#     mask = None
#     for range_dict in COLOR_RANGES[color_name]:
#         m = cv2.inRange(hsv, range_dict['lower'], range_dict['upper'])
#         mask = m if mask is None else cv2.bitwise_or(mask, m)
#     return mask


# def identify_shape(contour):
#     """识别形状"""
#     peri = cv2.arcLength(contour, True)
#     approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
#     vertices = len(approx)
    
#     if vertices == 3:
#         return 'triangle'
#     elif vertices == 4:
#         x, y, w, h = cv2.boundingRect(approx)
#         ratio = w / float(h)
#         return 'square' if 0.85 <= ratio <= 1.15 else 'rectangle'
#     elif vertices == 5:
#         return 'pentagon'
#     elif vertices == 6:
#         return 'hexagon'
#     elif vertices > 6:
#         area = cv2.contourArea(contour)
#         circularity = 4 * np.pi * area / (peri ** 2) if peri > 0 else 0
#         return 'circle' if circularity > 0.75 else 'polygon'
#     return 'unknown'


# def detect_shapes(image, min_area=500):
#     """检测图像中的彩色形状"""
#     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#     results = []
    
#     for color_name in COLOR_RANGES:
#         mask = get_color_mask(hsv, color_name)
        
#         kernel = np.ones((5, 5), np.uint8)
#         mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
#         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
#         for contour in contours:
#             area = cv2.contourArea(contour)
#             if area < min_area:
#                 continue
            
#             shape = identify_shape(contour)
#             M = cv2.moments(contour)
#             if M['m00'] == 0:
#                 continue
#             cx = int(M['m10'] / M['m00'])
#             cy = int(M['m01'] / M['m00'])
            
#             results.append({
#                 'color': color_name,
#                 'shape': shape,
#                 'contour': contour,
#                 'center': (cx, cy),
#                 'area': area
#             })
    
#     return results


# # ===================== RealSense 配置 =====================
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# pipeline.start(config)
# align = rs.align(rs.stream.color)


# def get_frames():
#     """获取对齐的RGB和深度帧"""
#     frames = align.process(pipeline.wait_for_frames())
#     depth_frame = frames.get_depth_frame()
#     color_frame = frames.get_color_frame()
#     depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
#     return np.asanyarray(color_frame.get_data()), depth_frame, depth_intrin


# def get_3d_coord(x, y, depth_frame, depth_intrin):
#     """像素坐标转3D相机坐标"""
#     dis = depth_frame.get_distance(x, y)
#     coord = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], dis)
#     return coord, dis


# # ===================== 主程序 =====================
# if __name__ == '__main__':
#     print("[INFO] 颜色形状检测节点启动")
#     rospy.init_node("shape_detect", anonymous=True)
#     object_pub = rospy.Publisher("object_pose", ObjectInfo, queue_size=10)
#     object_info_msg = ObjectInfo()
    
#     try:
#         while not rospy.is_shutdown():
#             color_image, depth_frame, depth_intrin = get_frames()
#             canvas = color_image.copy()
            
#             # 检测形状
#             shapes = detect_shapes(color_image, min_area=800)
            
#             # 左上角显示检测数量
#             cv2.putText(canvas, f"Objects: {len(shapes)}", (10, 30),
#                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
#             for i, obj in enumerate(shapes):
#                 color_name = obj['color']
#                 shape_name = obj['shape']
#                 contour = obj['contour']
#                 cx, cy = obj['center']
                
#                 # 获取3D坐标
#                 coord, dis = get_3d_coord(cx, cy, depth_frame, depth_intrin)
                
#                 # 绘制轮廓（用对应颜色）
#                 cv2.drawContours(canvas, [contour], -1, COLOR_BGR[color_name], 2)
                
#                 # 绘制中心点
#                 cv2.circle(canvas, (cx, cy), 5, (255, 255, 255), -1)
                
#                 # 显示标签：编号 + 颜色 + 形状
#                 label = f"#{i+1} {color_name} {shape_name}"
#                 cv2.putText(canvas, label, (cx - 60, cy - 25),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_BGR[color_name], 2)
                
#                 # 显示3D坐标
#                 coord_text = f"({coord[0]:.2f}, {coord[1]:.2f}, {coord[2]:.2f})m"
#                 cv2.putText(canvas, coord_text, (cx - 60, cy + 25),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
#                 # 显示距离
#                 dis_text = f"Dist: {dis:.2f}m"
#                 cv2.putText(canvas, dis_text, (cx - 60, cy + 45),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                
#                 # 发布ROS消息
#                 object_info_msg.object_class = f"{color_name}_{shape_name}"
#                 object_info_msg.x = float(coord[0])
#                 object_info_msg.y = float(coord[1])
#                 object_info_msg.z = float(coord[2])
#                 rospy.loginfo(f"#{i+1} {color_name} {shape_name}: X={coord[0]:.3f}, Y={coord[1]:.3f}, Z={coord[2]:.3f}")
#                 object_pub.publish(object_info_msg)
            
#             cv2.namedWindow('Shape Detection', cv2.WINDOW_NORMAL)
#             cv2.imshow('Shape Detection', canvas)
#             if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
#                 break
                
#     finally:
#         pipeline.stop()
#         cv2.destroyAllWindows()
#         print("[INFO] 节点关闭")



#!/home/airlab1/miniconda3/envs/ros1/bin/python3
# -*- coding=UTF-8 -*-
"""
颜色形状检测节点（带瑕疵检测）
功能：
1. 检测画面中的彩色几何图形
2. 识别颜色、形状和3D位置
3. 检测物体表面瑕疵（杂质、污点、划痕等）
4. ROI区域过滤，排除背景干扰
"""
import rospy
import pyrealsense2 as rs
import numpy as np
import cv2
from vi_msgs.msg import ObjectInfo

# ===================== 配置参数 =====================
class Config:
    # ROI区域设置（排除背景）- 根据实际场景调整
    USE_ROI = True
    ROI_X = 100          # ROI左上角X
    ROI_Y = 80           # ROI左上角Y
    ROI_WIDTH = 440      # ROI宽度
    ROI_HEIGHT = 320     # ROI高度
    
    # 深度过滤（排除太远或太近的物体）
    USE_DEPTH_FILTER = True
    MIN_DEPTH = 0.15     # 最小深度（米）
    MAX_DEPTH = 0.60     # 最大深度（米）
    
    # 形状检测参数
    MIN_CONTOUR_AREA = 800    # 最小轮廓面积
    MAX_CONTOUR_AREA = 50000  # 最大轮廓面积
    
    # 瑕疵检测参数
    DEFECT_DETECTION = True
    DEFECT_MIN_AREA = 20         # 瑕疵最小面积
    DEFECT_MAX_AREA = 500        # 瑕疵最大面积（太大就不是瑕疵了）
    DEFECT_THRESHOLD = 30        # 瑕疵检测阈值
    HOMOGENEITY_THRESHOLD = 0.85 # 均匀性阈值（低于此值认为有瑕疵）


# ===================== 颜色定义 =====================
COLOR_RANGES = {
    'red': [
        {'lower': np.array([0, 120, 70]), 'upper': np.array([10, 255, 255])},
        {'lower': np.array([160, 120, 70]), 'upper': np.array([180, 255, 255])}
    ],
    'orange': [{'lower': np.array([11, 120, 70]), 'upper': np.array([25, 255, 255])}],
    'yellow': [{'lower': np.array([26, 120, 70]), 'upper': np.array([34, 255, 255])}],
    'green': [{'lower': np.array([35, 120, 70]), 'upper': np.array([85, 255, 255])}],
    'cyan': [{'lower': np.array([86, 120, 70]), 'upper': np.array([100, 255, 255])}],
    'blue': [{'lower': np.array([101, 120, 70]), 'upper': np.array([130, 255, 255])}],
    'purple': [{'lower': np.array([131, 120, 70]), 'upper': np.array([159, 255, 255])}],
}

COLOR_BGR = {
    'red': (0, 0, 255), 'orange': (0, 165, 255), 'yellow': (0, 255, 255),
    'green': (0, 255, 0), 'cyan': (255, 255, 0), 'blue': (255, 0, 0), 
    'purple': (255, 0, 255), 'defect': (0, 0, 139),  # 深红色标记瑕疵
}


# ===================== 辅助函数 =====================
def get_color_mask(hsv, color_name):
    """获取指定颜色的掩码"""
    mask = None
    for range_dict in COLOR_RANGES[color_name]:
        m = cv2.inRange(hsv, range_dict['lower'], range_dict['upper'])
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def identify_shape(contour):
    """识别形状"""
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
    vertices = len(approx)
    
    if vertices == 3:
        return 'triangle'
    elif vertices == 4:
        x, y, w, h = cv2.boundingRect(approx)
        ratio = w / float(h)
        return 'square' if 0.85 <= ratio <= 1.15 else 'rectangle'
    elif vertices == 5:
        return 'pentagon'
    elif vertices == 6:
        return 'hexagon'
    elif vertices > 6:
        area = cv2.contourArea(contour)
        circularity = 4 * np.pi * area / (peri ** 2) if peri > 0 else 0
        return 'circle' if circularity > 0.75 else 'polygon'
    return 'unknown'


def is_in_roi(cx, cy, roi_offset=(0, 0)):
    """检查点是否在ROI区域内"""
    if not Config.USE_ROI:
        return True
    
    # 转换回原图坐标
    orig_x = cx + roi_offset[0]
    orig_y = cy + roi_offset[1]
    
    return (Config.ROI_X <= orig_x <= Config.ROI_X + Config.ROI_WIDTH and
            Config.ROI_Y <= orig_y <= Config.ROI_Y + Config.ROI_HEIGHT)


def check_depth_valid(depth, x, y, depth_frame):
    """检查深度是否有效"""
    if not Config.USE_DEPTH_FILTER:
        return True
    
    dis = depth_frame.get_distance(x, y)
    return Config.MIN_DEPTH <= dis <= Config.MAX_DEPTH


# ===================== 瑕疵检测 =====================
class DefectDetector:
    """瑕疵检测器"""
    
    @staticmethod
    def detect_defects(image, contour, color_name):
        """
        检测物体表面瑕疵
        
        Args:
            image: BGR图像
            contour: 物体轮廓
            color_name: 物体颜色名称
            
        Returns:
            defects: 瑕疵列表，每个瑕疵包含位置和类型
            quality_score: 质量评分 (0-100)
            defect_mask: 瑕疵掩码图
        """
        defects = []
        
        # 创建物体掩码
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        
        # 腐蚀掩码，排除边缘
        kernel = np.ones((5, 5), np.uint8)
        mask_eroded = cv2.erode(mask, kernel, iterations=2)
        
        # 提取物体区域
        object_region = cv2.bitwise_and(image, image, mask=mask_eroded)
        
        # 转换到不同颜色空间进行分析
        gray = cv2.cvtColor(object_region, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(object_region, cv2.COLOR_BGR2HSV)
        
        # 方法1：基于颜色异常检测杂质
        defect_mask_color = DefectDetector._detect_color_anomaly(
            hsv, mask_eroded, color_name
        )
        
        # 方法2：基于灰度变化检测污点/划痕
        defect_mask_texture = DefectDetector._detect_texture_anomaly(
            gray, mask_eroded
        )
        
        # 合并瑕疵掩码
        defect_mask = cv2.bitwise_or(defect_mask_color, defect_mask_texture)
        
        # 清理噪声
        defect_mask = cv2.morphologyEx(defect_mask, cv2.MORPH_OPEN, 
                                        np.ones((3, 3), np.uint8))
        defect_mask = cv2.morphologyEx(defect_mask, cv2.MORPH_CLOSE, 
                                        np.ones((3, 3), np.uint8))
        
        # 查找瑕疵轮廓
        defect_contours, _ = cv2.findContours(
            defect_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        for dc in defect_contours:
            area = cv2.contourArea(dc)
            if Config.DEFECT_MIN_AREA <= area <= Config.DEFECT_MAX_AREA:
                M = cv2.moments(dc)
                if M['m00'] > 0:
                    dcx = int(M['m10'] / M['m00'])
                    dcy = int(M['m01'] / M['m00'])
                    
                    # 判断瑕疵类型
                    defect_type = DefectDetector._classify_defect(dc, area)
                    
                    defects.append({
                        'center': (dcx, dcy),
                        'contour': dc,
                        'area': area,
                        'type': defect_type
                    })
        
        # 计算质量评分
        object_area = cv2.contourArea(contour)
        defect_area = sum(d['area'] for d in defects)
        defect_ratio = defect_area / object_area if object_area > 0 else 0
        
        # 评分：100分 - 瑕疵扣分
        quality_score = max(0, 100 - defect_ratio * 500 - len(defects) * 5)
        
        return defects, quality_score, defect_mask
    
    @staticmethod
    def _detect_color_anomaly(hsv, mask, expected_color):
        """基于颜色检测异常（杂质）"""
        defect_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        
        # 获取期望颜色的掩码
        expected_mask = get_color_mask(hsv, expected_color)
        
        # 物体区域中不属于期望颜色的部分
        # 就是可能的杂质
        anomaly = cv2.bitwise_and(mask, cv2.bitwise_not(expected_mask))
        
        # 排除黑色区域（可能是阴影）
        h, s, v = cv2.split(hsv)
        bright_mask = cv2.inRange(v, 30, 255)
        anomaly = cv2.bitwise_and(anomaly, bright_mask)
        
        return anomaly
    
    @staticmethod
    def _detect_texture_anomaly(gray, mask):
        """基于纹理检测异常（污点、划痕）"""
        defect_mask = np.zeros(gray.shape, dtype=np.uint8)
        
        # 只处理掩码内的区域
        masked_gray = cv2.bitwise_and(gray, gray, mask=mask)
        
        if cv2.countNonZero(mask) == 0:
            return defect_mask
        
        # 计算局部均值
        blur = cv2.GaussianBlur(masked_gray, (15, 15), 0)
        
        # 计算与局部均值的差异
        diff = cv2.absdiff(masked_gray, blur)
        
        # 阈值化找出异常区域
        _, defect_mask = cv2.threshold(
            diff, Config.DEFECT_THRESHOLD, 255, cv2.THRESH_BINARY
        )
        
        # 只保留物体内部的瑕疵
        defect_mask = cv2.bitwise_and(defect_mask, mask)
        
        return defect_mask
    
    @staticmethod
    def _classify_defect(contour, area):
        """分类瑕疵类型"""
        # 计算轮廓特征
        peri = cv2.arcLength(contour, True)
        
        if peri == 0:
            return 'unknown'
        
        circularity = 4 * np.pi * area / (peri ** 2)
        
        # 获取最小外接矩形
        rect = cv2.minAreaRect(contour)
        width, height = rect[1]
        if width == 0 or height == 0:
            return 'spot'
        
        aspect_ratio = max(width, height) / min(width, height)
        
        # 根据形状特征分类
        if circularity > 0.7:
            return 'spot'       # 污点（圆形）
        elif aspect_ratio > 3:
            return 'scratch'    # 划痕（长条形）
        else:
            return 'impurity'   # 杂质（不规则形状）


# ===================== 形状检测 =====================
def detect_shapes(image, depth_frame=None, roi_offset=(0, 0)):
    """检测图像中的彩色形状"""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    results = []
    
    for color_name in COLOR_RANGES:
        mask = get_color_mask(hsv, color_name)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # 面积过滤
            if area < Config.MIN_CONTOUR_AREA or area > Config.MAX_CONTOUR_AREA:
                continue
            
            shape = identify_shape(contour)
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # 深度过滤
            if depth_frame is not None and Config.USE_DEPTH_FILTER:
                orig_x = cx + roi_offset[0]
                orig_y = cy + roi_offset[1]
                dis = depth_frame.get_distance(orig_x, orig_y)
                if not (Config.MIN_DEPTH <= dis <= Config.MAX_DEPTH):
                    continue
            
            # 瑕疵检测
            defects = []
            quality_score = 100.0
            defect_mask = None
            
            if Config.DEFECT_DETECTION:
                defects, quality_score, defect_mask = DefectDetector.detect_defects(
                    image, contour, color_name
                )
            
            results.append({
                'color': color_name,
                'shape': shape,
                'contour': contour,
                'center': (cx, cy),
                'area': area,
                'defects': defects,
                'quality_score': quality_score,
                'defect_mask': defect_mask
            })
    
    return results


# ===================== 可视化 =====================
def draw_results(canvas, shapes, roi_offset=(0, 0)):
    """绘制检测结果"""
    for i, obj in enumerate(shapes):
        color_name = obj['color']
        shape_name = obj['shape']
        contour = obj['contour']
        cx, cy = obj['center']
        defects = obj['defects']
        quality_score = obj['quality_score']
        
        # 调整轮廓坐标（加上ROI偏移）
        adjusted_contour = contour + np.array([roi_offset[0], roi_offset[1]])
        adjusted_cx = cx + roi_offset[0]
        adjusted_cy = cy + roi_offset[1]
        
        # 根据质量评分选择轮廓颜色
        if quality_score >= 90:
            contour_color = COLOR_BGR[color_name]
            status = "OK"
            status_color = (0, 255, 0)
        elif quality_score >= 70:
            contour_color = (0, 165, 255)  # 橙色警告
            status = "WARN"
            status_color = (0, 165, 255)
        else:
            contour_color = (0, 0, 255)    # 红色不合格
            status = "NG"
            status_color = (0, 0, 255)
        
        # 绘制物体轮廓
        cv2.drawContours(canvas, [adjusted_contour], -1, contour_color, 2)
        
        # 绘制中心点
        cv2.circle(canvas, (adjusted_cx, adjusted_cy), 5, (255, 255, 255), -1)
        
        # 绘制瑕疵
        for defect in defects:
            dcx, dcy = defect['center']
            dcx += roi_offset[0]
            dcy += roi_offset[1]
            defect_contour = defect['contour'] + np.array([roi_offset[0], roi_offset[1]])
            
            # 用红色圈出瑕疵
            cv2.drawContours(canvas, [defect_contour], -1, (0, 0, 255), 2)
            cv2.circle(canvas, (dcx, dcy), 3, (0, 0, 255), -1)
            
            # 标注瑕疵类型
            defect_type_cn = {
                'spot': '污点',
                'scratch': '划痕', 
                'impurity': '杂质',
                'unknown': '异常'
            }
            cv2.putText(canvas, defect_type_cn.get(defect['type'], '?'),
                       (dcx + 5, dcy - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.4, (0, 0, 255), 1)
        
        # 显示标签
        label = f"#{i+1} {color_name} {shape_name}"
        cv2.putText(canvas, label, (adjusted_cx - 60, adjusted_cy - 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_BGR[color_name], 2)
        
        # 显示质量评分和状态
        quality_text = f"Q:{quality_score:.0f}% [{status}]"
        cv2.putText(canvas, quality_text, (adjusted_cx - 60, adjusted_cy - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
        
        # 如果有瑕疵，显示瑕疵数量
        if len(defects) > 0:
            defect_text = f"Defects: {len(defects)}"
            cv2.putText(canvas, defect_text, (adjusted_cx - 60, adjusted_cy + 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
    
    return canvas


def draw_roi(canvas):
    """绘制ROI区域"""
    if Config.USE_ROI:
        cv2.rectangle(canvas, 
                     (Config.ROI_X, Config.ROI_Y),
                     (Config.ROI_X + Config.ROI_WIDTH, Config.ROI_Y + Config.ROI_HEIGHT),
                     (0, 255, 255), 2)
        cv2.putText(canvas, "ROI", (Config.ROI_X + 5, Config.ROI_Y + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return canvas


# ===================== RealSense 配置 =====================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)


def get_frames():
    """获取对齐的RGB和深度帧"""
    frames = align.process(pipeline.wait_for_frames())
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    return np.asanyarray(color_frame.get_data()), depth_frame, depth_intrin


def get_3d_coord(x, y, depth_frame, depth_intrin):
    """像素坐标转3D相机坐标"""
    dis = depth_frame.get_distance(x, y)
    coord = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], dis)
    return coord, dis


# ===================== 主程序 =====================
if __name__ == '__main__':
    print("=" * 60)
    print("[INFO] 颜色形状检测节点启动（带瑕疵检测）")
    print("=" * 60)
    print(f"[CONFIG] ROI区域: {'启用' if Config.USE_ROI else '禁用'}")
    print(f"[CONFIG] 深度过滤: {'启用' if Config.USE_DEPTH_FILTER else '禁用'}")
    print(f"[CONFIG] 瑕疵检测: {'启用' if Config.DEFECT_DETECTION else '禁用'}")
    print("=" * 60)
    
    rospy.init_node("shape_detect", anonymous=True)
    object_pub = rospy.Publisher("object_pose", ObjectInfo, queue_size=10)
    object_info_msg = ObjectInfo()
    
    try:
        while not rospy.is_shutdown():
            color_image, depth_frame, depth_intrin = get_frames()
            canvas = color_image.copy()
            
            # 确定处理区域
            if Config.USE_ROI:
                roi_x, roi_y = Config.ROI_X, Config.ROI_Y
                roi_w, roi_h = Config.ROI_WIDTH, Config.ROI_HEIGHT
                roi_image = color_image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
                roi_offset = (roi_x, roi_y)
            else:
                roi_image = color_image
                roi_offset = (0, 0)
            
            # 检测形状（在ROI内）
            shapes = detect_shapes(roi_image, depth_frame, roi_offset)
            
            # 绘制ROI区域
            canvas = draw_roi(canvas)
            
            # 绘制检测结果
            canvas = draw_results(canvas, shapes, roi_offset)
            
            # 左上角显示统计信息
            cv2.putText(canvas, f"Objects: {len(shapes)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 统计瑕疵物体数量
            defect_count = sum(1 for s in shapes if len(s['defects']) > 0)
            if defect_count > 0:
                cv2.putText(canvas, f"Defective: {defect_count}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # 处理每个检测到的物体
            for i, obj in enumerate(shapes):
                color_name = obj['color']
                shape_name = obj['shape']
                cx, cy = obj['center']
                defects = obj['defects']
                quality_score = obj['quality_score']
                
                # 转换回原图坐标获取3D位置
                orig_cx = cx + roi_offset[0]
                orig_cy = cy + roi_offset[1]
                
                coord, dis = get_3d_coord(orig_cx, orig_cy, depth_frame, depth_intrin)
                
                # 显示3D坐标
                coord_text = f"({coord[0]:.2f}, {coord[1]:.2f}, {coord[2]:.2f})m"
                cv2.putText(canvas, coord_text, (orig_cx - 60, orig_cy + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # 日志输出
                defect_info = f", 瑕疵:{len(defects)}" if defects else ""
                rospy.loginfo(f"#{i+1} {color_name} {shape_name}: "
                            f"X={coord[0]:.3f}, Y={coord[1]:.3f}, Z={coord[2]:.3f}, "
                            f"质量={quality_score:.0f}%{defect_info}")
                
                # 发布ROS消息（可以扩展消息类型包含瑕疵信息）
                object_info_msg.object_class = f"{color_name}_{shape_name}"
                object_info_msg.x = float(coord[0])
                object_info_msg.y = float(coord[1])
                object_info_msg.z = float(coord[2])
                object_pub.publish(object_info_msg)
            
            # 显示窗口
            cv2.namedWindow('Shape Detection with Defect', cv2.WINDOW_NORMAL)
            cv2.imshow('Shape Detection with Defect', canvas)
            
            key = cv2.waitKey(1) & 0xFF
            if key in [ord('q'), 27]:  # q或ESC退出
                break
            elif key == ord('r'):  # r键重置ROI
                print("[INFO] 请拖动鼠标选择ROI区域...")
            elif key == ord('d'):  # d键切换瑕疵检测
                Config.DEFECT_DETECTION = not Config.DEFECT_DETECTION
                print(f"[INFO] 瑕疵检测: {'启用' if Config.DEFECT_DETECTION else '禁用'}")
                
    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] 节点关闭")