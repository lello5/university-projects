import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import argparse
from tqdm import tqdm 

class HParams():
    # benchmark_name: used to draw the obstacles
    benchmark_name = ''
    # borders of the environment
    x_lim = (-3, 3)
    y_lim = (-3, 3)
    # fraction to enlarge image frame
    frame_frac = 1.0
    
hparams = HParams()

# draw three point turn scenario (from the paper)
def draw_turn():
    hparams.x_lim = (0, 3)
    hparams.y_lim = (0, 1)
    
# draw driver test scenario (from the paper)
def draw_real_parking():
    hparams.x_lim = (0, 3)
    hparams.y_lim = (0, 3)
    
# classes used to draw the search tree
class Node():
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y

class NodeList():
    def __init__(self):
        self.nodes = []
        self.n_nodes = 0

    def addNode(self, id, x, y):
        self.nodes.append(Node(id, x, y))
        self.n_nodes += 1

    def getNodeById(self, id):
        for node in self.nodes:
            if node.id == id:
                return node

class Edge():
    def __init__(self, id, v1, v2):
        self.id = id
        self.v1 = v1
        self.v2 = v2

class EdgeList():
    def __init__(self):
        self.edges = []
        self.n_edges = 0

    def addEdge(self, id, v1, v2):
        self.edges.append(Edge(id, v1, v2))
        self.n_edges += 1

    def getEdgeById(self, id):
        for edge in self.edges:
            if edge.id == id:
                return edge

    def getNodesInEdge(self, id):
        edge = self.getEdgeById(id)
        return edge.v1, edge.v2
     
def draw_tree(graphml, outfile, exp):
    print('Parsing graphml file...')
    tree = ET.parse(graphml)
    root = tree.getroot()

    graph = root[2]
    nodes = NodeList()
    edges = EdgeList()

    for elem in tqdm(graph):
        if ('node' in str(elem.tag)):
            id = elem.attrib['id']
            for data in elem:
                x, y, _, _, _ = data.text.split(',')
                x = float(x)
                y = float(y)
                nodes.addNode(id, x, y)

        if ('edge' in str(elem.tag)):
            id, v1, v2 = elem.attrib['id'], elem.attrib['source'], elem.attrib['target']
            edges.addEdge(id, v1, v2)
    
    if exp != '':
        hparams.benchmark_name = exp
    # set limits for the environment
    if hparams.benchmark_name == 'turn':
        draw_turn()
    if hparams.benchmark_name == 'real':
        draw_real_parking()

    # adjust visible area
    plt.xlim([hparams.frame_frac*hparams.x_lim[0], hparams.frame_frac*hparams.x_lim[1]])
    plt.ylim([hparams.frame_frac*hparams.y_lim[0], hparams.frame_frac*hparams.y_lim[1]])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.axis('off')
    
    for edge in tqdm(edges.edges):
        v1, v2 = edges.getNodesInEdge(edge.id)
        n1 = nodes.getNodeById(v1)
        n2 = nodes.getNodeById(v2)
        plt.plot([n1.x, n2.x],
                 [n1.y, n2.y], color = 'green', linewidth=0.1, alpha=0.5) 
    
    print('States and edges plotted into the image.')

    plt.savefig(outfile, dpi=200, bbox_inches='tight', pad_inches=0)
    print('Final image saved.')
    
def main():
    parser = argparse.ArgumentParser()
   
    parser.add_argument(
        "--graphml",
        nargs='?',
        default='graph.graphml',
        type=str,
        help='Path to the input graphml file.'
    )
    parser.add_argument(
        "--outfile",
        nargs='?',
        default='tree.png',
        type=str,
        help='Path to the output png image.'
    )
    parser.add_argument(
        "--exp",
        nargs='?',
        default='',
        type=str,
        help='Type of experiment: {forward, backward, obstacles, turn, real}.'
    )
    args = parser.parse_args()
    
    print('Creating %s image from %s to %s' % (args.exp, args.graphml, args.outfile))
    
    draw_tree(args.graphml, args.outfile, args.exp) 
        
if __name__ == '__main__':
    main()
