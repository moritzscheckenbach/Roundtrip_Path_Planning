# coding: utf-8

"""
This code is part of the course 'Innovative Programmiermethoden für Industrieroboter' (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import networkx as nx

def visibilityPRM_custom_Visualize(planner, solution, ax = None, nodeSize = 300):
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    graph = planner.graph
    statsHandler = planner.statsHandler
    collChecker = planner._collisionChecker
    pos = nx.get_node_attributes(graph,'pos')
    color = nx.get_node_attributes(graph,'color')
    
    if 'start' in statsHandler.graph:
        statsHandler.graph.remove_node('start')
    if 'goal' in statsHandler.graph:
        statsHandler.graph.remove_node('goal')

    if statsHandler:
         statPos = nx.get_node_attributes(statsHandler.graph,'pos')
         print(f"StatsHandler: {statPos}")
         nx.draw(statsHandler.graph, pos=statPos, alpha=0.2,edge_color='y',node_size=nodeSize)
         print(f"Nodes im StatsHandler Graph {list(statsHandler.graph.nodes)}")
        
    # draw graph (nodes colorized by degree)
    nx.draw(graph, pos = pos, nodelist=color.keys(), node_color = color.values(), ax=ax)   
    nx.draw_networkx_edges(graph,pos,
                               edge_color='b',
                               width=3.0, ax=ax
                            )
   
    collChecker.drawObstacles(ax)
    # get nodes based on solution path
    Gsp = nx.subgraph(graph,solution)

    # draw edges based on solution path
    nx.draw_networkx_edges(Gsp,pos,alpha=0.8,edge_color='g',width=10, label="Solution Path",ax=ax)