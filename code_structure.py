import os
from graphviz import Digraph

def generate_folder_structure_graph(folder_path, graph=None):
    """
    生成文件夹结构的图形表示
    """
    if graph is None:
        graph = Digraph('FolderStructure', comment='Folder Structure Graph', format='png')
    for root, dirs, files in os.walk(folder_path):
        # 添加当前文件夹节点
        graph.node(root, label=os.path.basename(root))
        for dir_name in dirs:
            dir_path = os.path.join(root, dir_name)
            # 添加子文件夹节点，并连线到当前文件夹
            graph.node(dir_path, label=dir_name)
            graph.edge(root, dir_path)
        for file_name in files:
            file_path = os.path.join(root, file_name)
            # 添加文件节点，并连线到当前文件夹
            graph.node(file_path, label=file_name, shape='box')
            graph.edge(root, file_path)
    return graph

if __name__ == "__main__":
    folder_path = r'C:\your\folder\path'  # 这里替换成你实际要展示结构的文件夹路径
    graph = generate_folder_structure_graph(folder_path)
    graph.render('folder_structure', view=True)
