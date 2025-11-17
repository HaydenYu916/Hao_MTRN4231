#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试OpenCV显示功能
用于诊断可视化窗口无法显示的问题
"""

import cv2
import numpy as np
import sys
import os

def test_display():
    """测试OpenCV显示功能"""
    print("=" * 60)
    print("OpenCV 显示功能诊断工具")
    print("=" * 60)
    
    # 检查DISPLAY环境变量
    display = os.environ.get('DISPLAY')
    print(f"\n1. DISPLAY环境变量: {display if display else '未设置'}")
    
    # 检查OpenCV版本
    print(f"2. OpenCV版本: {cv2.__version__}")
    
    # 检查GUI后端
    try:
        backend = cv2.getBuildInformation()
        if 'GUI' in backend:
            print("3. OpenCV GUI支持: 已编译")
        else:
            print("3. OpenCV GUI支持: 未编译")
    except:
        print("3. OpenCV GUI支持: 无法检测")
    
    # 测试创建窗口
    print("\n4. 测试窗口创建...")
    window_name = '测试窗口'
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 480)
        print("   ✓ 窗口创建成功")
    except Exception as e:
        print(f"   ✗ 窗口创建失败: {e}")
        print("   提示: 可能需要GUI环境或X11转发")
        return False
    
    # 创建测试图像
    print("\n5. 创建测试图像...")
    test_image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(test_image, 'OpenCV Display Test', (50, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(test_image, 'Press any key to continue', (50, 300),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    print("   ✓ 测试图像创建成功")
    
    # 测试显示图像
    print("\n6. 测试图像显示...")
    try:
        cv2.imshow(window_name, test_image)
        print("   ✓ imshow() 调用成功")
        print("   如果看到窗口，说明显示功能正常")
        print("   如果看不到窗口，可能的原因：")
        print("   - 窗口被其他窗口遮挡")
        print("   - 在远程SSH连接中（需要X11转发: ssh -X）")
        print("   - DISPLAY环境变量未正确设置")
        print("   - 没有GUI环境（纯命令行环境）")
    except Exception as e:
        print(f"   ✗ imshow() 调用失败: {e}")
        cv2.destroyAllWindows()
        return False
    
    # 等待按键
    print("\n7. 等待用户交互...")
    print("   按任意键继续，或等待5秒后自动退出...")
    key = cv2.waitKey(5000) & 0xFF
    if key != 255:  # 255表示超时
        print(f"   ✓ 检测到按键: {chr(key) if key < 128 else '特殊键'}")
    else:
        print("   ⚠ 5秒内未检测到按键（可能窗口未显示）")
    
    # 清理
    cv2.destroyAllWindows()
    print("\n8. 清理完成")
    
    print("\n" + "=" * 60)
    print("诊断完成")
    print("=" * 60)
    
    return True

if __name__ == '__main__':
    try:
        success = test_display()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n用户中断")
        cv2.destroyAllWindows()
        sys.exit(0)
    except Exception as e:
        print(f"\n\n✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

