import numpy as np

def main():
    # 文件名，根据实际情况修改
    filename = "sampled_data_20250119000827.npy"
    # 使用 numpy 的 load 函数加载.npy 文件
    data = np.load(filename)
    print("Loaded data from", filename)
    print(data)

if __name__ == "__main__":
    main()
