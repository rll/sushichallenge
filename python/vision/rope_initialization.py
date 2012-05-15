import networkx as nx, numpy as np, scipy.spatial.distance as ssd
from collections import deque

def points_to_graph(xyzs, max_dist):
    pdists = ssd.squareform(ssd.pdist(xyzs))
    G = nx.Graph()
    for (i_from, row) in enumerate(pdists):
        G.add_node(i_from, xyz = xyzs[i_from])
        to_inds = np.flatnonzero(row[:i_from] < max_dist)
        for i_to in to_inds:
            G.add_edge(i_from, i_to, length = pdists[i_from, i_to])
    return G

def skeletonize_graph(G, resolution):
    G = largest_connected_component(G)
    calc_distances(G)
    partitions = calc_reeb_partitions(G,resolution)
    node2part = {}
    skel = nx.Graph()    
    for (i_part, part) in enumerate(partitions):
        xyzsum = np.zeros(3)
        for node in part:
            node2part[node] = i_part
            xyzsum += G.node[node]["xyz"]
        skel.add_node(i_part,xyz = xyzsum / len(part))
            
    for (node0, node1) in G.edges():
        if node2part[node0] != node2part[node1]:
            skel.add_edge(node2part[node0], node2part[node1])
        
    return skel

def largest_connected_component(G):
    sgs = nx.connected_component_subgraphs(G)
    return sgs[0]

def skeletonize_point_cloud(xyzs):
    G = points_to_graph(xyzs, .02)
    S = skeletonize_graph(G, .04)
    return S
        
def calc_distances(G):
    node0 = G.nodes_iter().next()
    G.node[node0]["dist"] = 0
    frontier = deque([node0])
    
    while len(frontier) > 0:
        node = frontier.popleft()
        node_dist = G.node[node]["dist"]
        for nei in G.neighbors_iter(node):
            nei_dist = G.node[nei].get("dist")
            if G.node[nei].get("dist") is None or G.edge[node][nei]["length"] + node_dist < nei_dist:
                frontier.append(nei)
                G.node[nei]["dist"] = G.edge[node][nei]["length"] + node_dist
        
def calc_reeb_partitions(G, resolution):
    nodes = G.nodes()
    distances = np.array([G.node[node]["dist"] for node in nodes])
    bin_edges = np.arange(distances.min()-1e-5, distances.max()+1e-5, resolution)
    bin_inds = np.searchsorted(bin_edges, distances) - 1
    dist_partitions =  [[] for _ in xrange(bin_inds.max() + 1)]
    for (i_node,i_part) in enumerate(bin_inds):
        dist_partitions[i_part].append(nodes[i_node])
        
    reeb_partitions = []
    for part in dist_partitions:
        sg = G.subgraph(part)
        reeb_partitions.extend(nx.connected_components(sg))
    return reeb_partitions
    
    