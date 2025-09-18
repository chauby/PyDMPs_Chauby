# -*- coding: utf-8 -*-
"""
NGGD模仿学习算法 - 依赖安装脚本
自动安装所需的Python包
"""

import subprocess
import sys
import os

def install_package(package):
    """安装Python包"""
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", package])
        print(f"✓ 成功安装 {package}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ 安装 {package} 失败: {e}")
        return False

def check_package(package):
    """检查包是否已安装"""
    try:
        __import__(package)
        return True
    except ImportError:
        return False

def main():
    """主安装函数"""
    print("=" * 60)
    print("NGGD模仿学习算法 - 依赖安装")
    print("=" * 60)
    
    # 必需的包列表
    required_packages = [
        ("numpy", "numpy"),
        ("scipy", "scipy"),
        ("matplotlib", "matplotlib"),
        ("sklearn", "scikit-learn"),
        ("pandas", "pandas")
    ]
    
    print("检查已安装的包...")
    
    missing_packages = []
    for import_name, package_name in required_packages:
        if check_package(import_name):
            print(f"✓ {package_name} 已安装")
        else:
            print(f"✗ {package_name} 未安装")
            missing_packages.append(package_name)
    
    if not missing_packages:
        print("\n🎉 所有依赖包都已安装！")
        return True
    
    print(f"\n需要安装 {len(missing_packages)} 个包:")
    for package in missing_packages:
        print(f"  - {package}")
    
    # 询问是否安装
    response = input("\n是否自动安装缺失的包? (y/n): ")
    if response.lower() != 'y':
        print("安装已取消")
        return False
    
    print("\n开始安装...")
    
    success_count = 0
    for package in missing_packages:
        if install_package(package):
            success_count += 1
    
    print(f"\n安装完成: {success_count}/{len(missing_packages)} 个包安装成功")
    
    if success_count == len(missing_packages):
        print("🎉 所有依赖包安装成功！")
        return True
    else:
        print("⚠️  部分包安装失败，请手动安装")
        return False

if __name__ == "__main__":
    main()
