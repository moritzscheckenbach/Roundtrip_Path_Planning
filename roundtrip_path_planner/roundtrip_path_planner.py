# coding: utf-8

"""
This is the main file for the roundtrip path planner.

Args:
startpos (array): start position in configuration space
targetlist (list): list of goal positions (arrays) in configuration space
environment: the environment in which the path should be planned

"""

################################
# Import the necessary libraries
################################

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import heapq
import math

import IPAStar
import IPBasicPRM
import IPLazyPRM
import IPPRMBase
import IPRRT
import IPVisibilityPRM

import IPVISBasicPRM
import IPVISLazyPRM
import IPVISVisibilityPRM
import IPVISAStar
import IPVISRRT

from IPPerfMonitor import IPPerfMonitor


###########################
# structure for the results
###########################

class ResultCollection (object):
    
    def __init__(self, plannerFactoryName, planner, benchmark, solution, perfDataFrame):
        self.plannerFactoryName = plannerFactoryName
        self.planner = planner
        self.benchmark = benchmark
        self.solution = solution
        self.perfDataFrame = perfDataFrame


########################
# Roundtrip Path Planner
########################

class Roundtrip_Path_Planner:
    def __init__(self, startpos, targetlist, environment, plannerName):
        
        #######################
        # Planner Configuration
        #######################

        """
        An dieser Stelle müssen für alle Algorithmen die Konfigurationen vordefiniert werden,
        sodass sie später in der GUI ausgewählt werden können und keine weiteren Parameter
        mehr nötig sind.

        Kopiert aus IP-X-0-Automated_PlanerTest.ipynb
        """

        supportedPlanners = dict()

        basicConfig = dict()
        basicConfig["radius"] = 3
        basicConfig["numNodes"] = 200
        supportedPlanners["basePRM"] = [IPBasicPRM.BasicPRM, basicConfig, IPVISBasicPRM.basicPRMVisualize]

        basicConfig2 = dict()
        basicConfig2["radius"] = 6
        basicConfig2["numNodes"] = 600
        supportedPlanners["basePRM2"] = [IPBasicPRM.BasicPRM, basicConfig2, IPVISBasicPRM.basicPRMVisualize]

        visbilityConfig = dict()
        visbilityConfig["ntry"] = 300
        supportedPlanners["visibilityPRM"] = [IPVisibilityPRM.VisPRM, visbilityConfig, IPVISVisibilityPRM.visibilityPRMVisualize ]

        # kClosestConfig = dict()
        # kClosestConfig["k"] = 7
        # kClosestConfig["numNodes"] = 300
        # supportedPlanners["kClosestPRM"] = [IPKClosestPRM.KClosestPRM, kClosestConfig, IPVISBasicPRM.basicPRMVisualize]

        lazyConfig = dict()
        lazyConfig["initialRoadmapSize"] = 10
        lazyConfig["updateRoadmapSize"]  = 5 
        lazyConfig["kNearest"] = 8
        supportedPlanners["lazyPRM"] = [IPLazyPRM.LazyPRM, lazyConfig, IPVISLazyPRM.lazyPRMVisualize]

        astarConfig = dict()
        astarConfig["heuristic"] = 'euclidean' 
        astarConfig["w"]  = 0.5
        supportedPlanners["astar"] = [IPAStar.AStar, astarConfig, IPVISAStar.aStarVisualize]

        # astarConfig2 = dict()
        # astarConfig2["heuristic"] = 'euclidean' 
        # astarConfig2["w"]  = 0.9
        # supportedPlanners["astar2"] = [IPAStar.AStar, astarConfig2, IPVISAStar.aStarVisualize]

        # rrtSimpleConfig = dict()
        # rrtSimpleConfig["numberOfGeneratedNodes"] = 100 
        # rrtSimpleConfig["testGoalAfterNumberOfNodes"]  = 10
        # supportedPlanners["simpleRRT"] = [IPRRT.RRTSimple, rrtSimpleConfig, IPVISRRT.rrtPRMVisualize]
        
        # Initialize all necessary members
        self.startpos = startpos # start position in configuration space
        self.targetlist = targetlist # list of goal positions in configuration space
        self.environment = environment # reference to Environment
        self.plannerName = plannerName # reference to algorithm
        self.config = supportedPlanners # dictionary with the needed information about the algorithms configuration options

    def plan(self):
        resultList = list()
        key = self.plannerName # key enthält den Nemen des Planers
        producer = self.config[key] # producer enthält alle items der config des Planers
        print(key, producer) # Key ist der Planer, producer ist der Konstruktor (Vorher definierte Configs [siehe oben])

        print ("Planning: " + key + " - " + self.environment.name)
        planner = producer[0](self.environment.collisionChecker) # Aufruf erstes Element aus producer (Klasse des Planers) und Aufruf der Methode Benchmark.collisionChecker
        IPPerfMonitor.clearData() # Löscht die Daten aus dem Dataframe des Performance Monitors
        
        """TBD: Evtl. Anpassung des Auswahlverfahrens für das nächste Ziel"""
        usedstart = self.startpos[0]
        goals = self.targetlist
        pastgoals = []

        for i in range(len(goals)):
            min_distance = None
            min_distance = float('inf')

            for goal in goals:
                if goal not in pastgoals:
                    distance = math.sqrt((usedstart[0] - goal[0]) ** 2 + (usedstart[1] - goal[1]) ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        goal_gespeichert = goal
            
            # Aktualisieren des Startpunktes und der Liste der bereits besuchten Punkte
            usedstart = goal_gespeichert # Alter Zielpunkt wird zum neuen Startpunkt
            pastgoals.append(goal_gespeichert) # Zielpunkt wird in die Liste der bereits besuchten Punkte eingetragen

        # Pfadplanung für die besuchten Ziele
        usedstart = self.startpos[0]
        composed_graph = nx.Graph()

        for i in range(len(pastgoals)):
            print(f"Pastgoals: {pastgoals}")
            print(f"Usedstart: {usedstart}")
            tmp_1 = []
            tmp_1.append(usedstart)
            tmp_2 = []
            tmp_2.append(pastgoals[i])
            
            print(f"tmp_1: {tmp_1}")
            print(f"tmp_2: {tmp_2}")

            try:
                resultList.append(ResultCollection(key,
                                                planner, 
                                                self.environment, 
                                                planner.planPath(tmp_1,tmp_2,producer[1]), # Aufruf der Methode planPath des Planers
                                                IPPerfMonitor.dataFrame()
                                                ))
                
                # Visualisierung der Ergebnisse
                fig_local = plt.figure(figsize=(10,10))
                ax = fig_local.add_subplot(1,1,1)
                title = self.plannerName + " - " + resultList[i].benchmark.name
                if resultList[i].solution == []:
                    title += " (No path found!)"
                title += "\n Assumed complexity level " + str(resultList[i].benchmark.level)
                ax.set_title(title)
                try:
                    if not resultList[i].solution or len(resultList[i].solution) < 2:
                        print(f"Invalid solution for planner {key}: {resultList[i].solution}")
                        continue

                    self.config[resultList[i].plannerFactoryName][2](resultList[i].planner, resultList[i].solution, ax=ax, nodeSize=100)
                    
                    # Speichern des Graphen
                    composed_graph = nx.compose(resultList[i].planner.graph, composed_graph)

                except Exception as e:
                    print (f"Visualizing error for planner {key}: {e}")
                    print(f"Exception details: {e}")
                    pass

            except Exception as e:
                print ("PLANNING ERROR ! PLANNING ERROR ! PLANNING ERROR ")
                print(f"Exception details: {e}")
                pass
            usedstart = pastgoals[i]
            print(f"New Usedstart: {usedstart}")

        print(f"Resultlist solution: {resultList[0].solution}")
        print(f"Resultlist solution: {resultList[1].solution}")

        whole_solution = []
        whole_solution.append(f"-{self.startpos[0][0]}-{self.startpos[0][1]}-")
        for i in range(len(resultList)):
            if resultList[i].solution == []:
                continue
            whole_solution.extend(resultList[i].solution[1:-1])
            whole_solution.append(f"-{pastgoals[i][0]}-{pastgoals[i][1]}-")

        print(f"whole solution: {whole_solution}")

        # # Visualisierung des gesamten zusammengesetzten Graphen
        # fig_composed = plt.figure(figsize=(10,10))
        # ax_composed = fig_composed.add_subplot(1,1,1)
        # ax_composed.set_title("Composed Graph")
        # pos = {node: (node[0], node[1]) for node in composed_graph.nodes()}
        # nx.draw(composed_graph, pos, ax=ax_composed, with_labels=True, node_size=100, node_color='blue', edge_color='gray')
        # plt.show()

        # Visualisierung des gesamten Pfades
        # Umwandeln der Knoten-Namen in Koordinaten
        coordinates = []
        for node in whole_solution:
            if node.startswith('-') and node.endswith('-'):
                parts = node.strip('-').split('-')
                coordinates.append((int(parts[0]), int(parts[1])))
            else:
                coordinates.append(node)

        # Visualisierung des Environments
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_title("Solution Path with Environment")

        # Zeichnen der Hindernisse im Environment
        planner._collisionChecker.drawObstacles(ax)

        # Zeichnen des Pfads der Lösung
        path_x = [coord[0] for coord in coordinates]
        path_y = [coord[1] for coord in coordinates]
        ax.plot(path_x, path_y, 'go-', label='Solution Path', zorder=2)  # 'go-' steht für grüne Punkte und Linien

        # Hervorheben des Start- und Zielpunkts
        ax.plot(coordinates[0][0], coordinates[0][1], 'bo', markersize=10, label='Start', zorder=3)  # 'bo' steht für blauer Punkt
        ax.plot(coordinates[-1][0], coordinates[-1][1], 'ro', markersize=10, label='Goal', zorder=3)  # 'ro' steht für roter Punkt

        # Setzen der Achsenbeschriftung
        Env_Limits = planner._collisionChecker.getEnvironmentLimits()
        x_Limits = Env_Limits[0]
        y_Limits = Env_Limits[1]
        ax.set_xticks(range(int(x_Limits[0]), int(x_Limits[1]) + 1))
        ax.set_yticks(range(int(y_Limits[0]), int(y_Limits[1]) + 1))
        ax.legend()
        plt.show()
