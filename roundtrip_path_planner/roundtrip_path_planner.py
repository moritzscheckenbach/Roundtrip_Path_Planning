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
import pandas as pd

import IPAStar
import IPBasicPRM
import IPLazyPRM
import IPPRMBase
import IPRRT
import IPVisibilityPRM
import IPVisibilityPRM_Customized

import IPVISBasicPRM
import IPVISLazyPRM
import IPVISVisibilityPRM
import IPVISVisibilityPRM_Customized
import IPVISAStar
import IPVISRRT

from IPPerfMonitor import IPPerfMonitor
import os


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
    def __init__(self, startpos, targetlist, environment, plannerName, sortTargetlist=True):
        
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
        basicConfig["numNodes"] = 400
        supportedPlanners["basePRM"] = [IPBasicPRM.BasicPRM, basicConfig, IPVISBasicPRM.basicPRMVisualize]

        basicConfig2 = dict()
        basicConfig2["radius"] = 6
        basicConfig2["numNodes"] = 600
        supportedPlanners["basePRM2"] = [IPBasicPRM.BasicPRM, basicConfig2, IPVISBasicPRM.basicPRMVisualize]

        visbilityConfig = dict()
        visbilityConfig["ntry"] = 400
        supportedPlanners["visibilityPRM"] = [IPVisibilityPRM.VisPRM, visbilityConfig, IPVISVisibilityPRM.visibilityPRMVisualize]

        visbility_custom_Config = dict()
        visbility_custom_Config["ntry"] = 1000
        supportedPlanners["visibilityPRM_custom"] = [IPVisibilityPRM_Customized.VisPRM_Custom, visbility_custom_Config, IPVISVisibilityPRM_Customized.visibilityPRM_custom_Visualize]

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
        self.sortTargetlist = sortTargetlist # sort the targetlist by the shortest distance


    # def _getRoadmap(self, config):
    #     if self.plannerName == "basePRM":
    #         radius = config["radius"]
    #         numNodes = config["numNodes"]
    #         self._learnRoadmapNearestNeighbour(radius, numNodes)
    #     elif self.plannerName == "lazyPRM":
    #         initialRoadmapSize = config["initialRoadmapSize"]
    #         updateRoadmapSize = config["updateRoadmapSize"]
    #         kNearest = config["kNearest"]
    #         self._buildRoadmap(initialRoadmapSize, kNearest)
    #     elif self.plannerName == "visibilityPRM":
    #         ntry = config["ntry"]
    #         self._learnRoadmap(ntry)
    #     elif self.plannerName == "visibilityPRM_custom":
    #         ntry = config["ntry"]
    #         self._learnRoadmap(ntry)


    def plan_MQ(self):
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
        whole_solution = []
        whole_solution.append((self.startpos[0][0], self.startpos[0][1]))

        # Sortiere die Ziele nach der kürzesten Distanz
        if self.sortTargetlist:
            # Auswahl des nächsten Ziels
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
        
        # Sortierung der Ziele nach der Reihenfolge der Liste
        else:
            pastgoals = goals



####################################################

        try:
            final_graph = planner.createGraph([usedstart], pastgoals, producer[1])
            print(f"nach final graph")

        except:
            print("no multiquery compatibility right now")


#####################################################

        for i in range(len(pastgoals)):
            
            print(f"Startpos: {usedstart}")
            print(f"Goalpos: {pastgoals}")
            try:
                if i == 0:
                    resultList.append(ResultCollection(key,
                                planner, 
                                self.environment,
                                nx.shortest_path(final_graph,'start','goal_1'), # Aufruf der Methode createGraph des Planers
                                #planner.planPath([usedstart],[pastgoals[i]],producer[1]), # Aufruf der Methode planPath des Planers
                                IPPerfMonitor.dataFrame()
                                ))
                else:
                    resultList.append(ResultCollection(key,
                                planner, 
                                self.environment,
                                nx.shortest_path(final_graph, f'goal_{i}', f'goal_{i+1}'), # Aufruf der Methode createGraph des Planers
                                #planner.planPath([usedstart],[pastgoals[i]],producer[1]), # Aufruf der Methode planPath des Planers
                                IPPerfMonitor.dataFrame() 
                                ))
            
                # Visualisierung der Ergebnisse
                fig_local = plt.figure(figsize=(10,10))
                ax = fig_local.add_subplot(1,1,1)
                title = f"{self.plannerName} - {resultList[i].benchmark.name}"
                if resultList[i].solution == []:
                    title += " (No path found!)"
                title += "\n Assumed complexity level " + str(resultList[i].benchmark.level)
                ax.set_title(title)
                Env_Limits = planner._collisionChecker.getEnvironmentLimits()
                x_Limits = Env_Limits[0]
                y_Limits = Env_Limits[1]
                ax.set_xticks(range(int(x_Limits[0]), int(x_Limits[1]) + 1))
                ax.set_yticks(range(int(y_Limits[0]), int(y_Limits[1]) + 1))
                ax.set_xlim(0, 22)
                ax.set_ylim(0, 22)
                ax.set_xlabel('X-Achse')
                ax.set_ylabel('Y-Achse')
                ax.grid(True)

                # Save Solution in whole_solution as tuple of x and y coordinates
                graph = resultList[i].planner.graph
                solution = resultList[i].solution[1:-1]  # Ignoriere den ersten und letzten Knoten (start und goal)
                # Füge die formatierte Version der SolutionNode hinzu
                for node in solution:
                    if 'pos' in graph.nodes[node]:
                        x = graph.nodes[node]['pos'][0]
                        y = graph.nodes[node]['pos'][1]
                        whole_solution.append((x, y))
                
                # Füge das Ziel der aktuellen Lösung hinzu
                whole_solution.append((pastgoals[i][0], pastgoals[i][1]))
                print(f"whole solution after adding goal: {whole_solution}")

                try:
                    self.config[resultList[i].plannerFactoryName][2](resultList[i].planner, resultList[i].solution, ax=ax, nodeSize=100)
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


        vollstaendiger_pfad = True

        for i in range(len(resultList)):
            print(f"ResultList: {resultList[i]}")
            if resultList[i].solution == []:
                vollstaendiger_pfad = False
                break

        if vollstaendiger_pfad:
            try:
                print(f"Final whole solution: {whole_solution}")

                # Visualisierung des gesamten Pfades
                # Umwandeln der Knoten-Namen in Koordinaten
                coordinates = whole_solution

                # Visualisierung des Environments
                fig, ax = plt.subplots(figsize=(10, 10))
                ax.set_title("Solution Path with Environment")

                # Zeichnen der Hindernisse im Environment
                planner._collisionChecker.drawObstacles(ax)

                # Zeichnen des Pfads der Lösung
                path_x = [coord[0] for coord in coordinates]
                path_y = [coord[1] for coord in coordinates]
                ax.plot(path_x, path_y, 'go-', label='Solution Path', zorder=2)  # 'go-' steht für grüne Punkte und Linien

                # Hervorheben des Startpunkts
                ax.plot(coordinates[0][0], coordinates[0][1], 'bo', markersize=10, label='Start', zorder=3)  # 'bo' steht für blauer Punkt

                # Hervorhebn der Zwischenziele
                for nodes in pastgoals[:-1]:
                    ax.plot(nodes[0], nodes[1], 'C6o', markersize=10, label='intermediate Goal', zorder=3)

                # Hervorheben des Zielpunkts
                ax.plot(coordinates[-1][0], coordinates[-1][1], 'ro', markersize=10, label='Goal', zorder=3)  # 'ro' steht für roter Punkt

                # Setzen der Achsenbeschriftung
                Env_Limits = planner._collisionChecker.getEnvironmentLimits()
                x_Limits = Env_Limits[0]
                y_Limits = Env_Limits[1]
                ax.set_xticks(range(int(x_Limits[0]), int(x_Limits[1]) + 1))
                ax.set_yticks(range(int(y_Limits[0]), int(y_Limits[1]) + 1))
                ax.legend()
                ax.grid(True)
                plt.show()
            except Exception as e:
                print(f"An error occurred: {e}")

        return resultList
    
    def plan(self):
        resultList = list()
        key = self.plannerName
        producer = self.config[key]
        print(key, producer)

        print("Planning: " + key + " - " + self.environment.name)
        planner = producer[0](self.environment.collisionChecker)
        IPPerfMonitor.clearData()

        usedstart = self.startpos[0]
        goals = self.targetlist
        pastgoals = []
        whole_solution = []

        if self.sortTargetlist:
            # Sortiere die Ziele nach der kürzesten Distanz
            for i in range(len(goals)):
                min_distance = float('inf')
                for goal in goals:
                    if goal not in pastgoals:
                        distance = math.sqrt((usedstart[0] - goal[0]) ** 2 + (usedstart[1] - goal[1]) ** 2)
                        if distance < min_distance:
                            min_distance = distance
                            goal_gespeichert = goal

                usedstart = goal_gespeichert
                pastgoals.append(goal_gespeichert)

            usedstart = self.startpos[0]
        
        else:
            pastgoals = goals
    
        for i in range(len(pastgoals)):
            print(f"Startpos: {usedstart}")
            print(f"Goal: {pastgoals[i]}")
            try:
                resultList.append(ResultCollection(key,
                                                planner,
                                                self.environment,
                                                planner.planPath([usedstart], [pastgoals[i]], producer[1]),
                                                IPPerfMonitor.dataFrame()
                                                ))
                usedstart = pastgoals[i]

                # Visualisierung der Ergebnisse
                fig_local = plt.figure(figsize=(10,10))
                ax = fig_local.add_subplot(1,1,1)
                title = f"{self.plannerName} - {resultList[i].benchmark.name}"
                if resultList[i].solution == []:
                    title += " (No path found!)"
                title += "\n Assumed complexity level " + str(resultList[i].benchmark.level)
                ax.set_title(title)
                Env_Limits = planner._collisionChecker.getEnvironmentLimits()
                x_Limits = Env_Limits[0]
                y_Limits = Env_Limits[1]
                ax.set_xticks(range(int(x_Limits[0]), int(x_Limits[1]) + 1))
                ax.set_yticks(range(int(y_Limits[0]), int(y_Limits[1]) + 1))
                ax.set_xlim(0, 22)
                ax.set_ylim(0, 22)
                ax.set_xlabel('X-Achse')
                ax.set_ylabel('Y-Achse')
                ax.grid(True)



                # Save Solution in whole_solution as tuple of x and y coordinates
                graph = resultList[i].planner.graph
                solution = resultList[i].solution
                if solution != []:
                    solution = solution [1:-1]  # Ignoriere den ersten und letzten Knoten (start und goal)
                    len_solution = len(solution)
                else:
                    len_solution = "No path found"
                # Füge die formatierte Version der SolutionNode hinzu
                for node in solution:
                    if 'pos' in graph.nodes[node]:
                        x = graph.nodes[node]['pos'][0]
                        y = graph.nodes[node]['pos'][1]
                        whole_solution.append((x, y))
                
                # save performance data in a pandas dataframe
                temp = {"name": [key],
                        "solution size": len_solution,
                        "time": [resultList[i].perfDataFrame.groupby(["name"]).sum(numeric_only=True)["time"]["planPath"]],
                        "graph_size": [planner.graph.size()]
                        }
                
                performance_dataframe = pd.DataFrame(temp)

                if not os.path.isfile("performance_data.csv"):
                    performance_dataframe.to_csv("performance_data.csv", mode='a', sep=',', header=True, index=False)
                else:
                    performance_dataframe.to_csv("performance_data.csv", mode='a', sep=',', header=False, index=False)

                # Füge das Ziel der aktuellen Lösung hinzu
                whole_solution.append((pastgoals[i][0], pastgoals[i][1]))
                print(f"whole solution after adding goal: {whole_solution}")

                try:
                    self.config[resultList[i].plannerFactoryName][2](resultList[i].planner, resultList[i].solution, ax=ax, nodeSize=100)
                except Exception as e:
                    print (f"Visualizing error for planner {key}: {e}")
                    print(f"Exception details: {e}")
                    pass
                
            except Exception as e:
                print("PLANNING ERROR ! PLANNING ERROR ! PLANNING ERROR\nSingle Query")
                print(f"Exception details: {e}")
                pass


        vollstaendiger_pfad = True

        for i in range(len(resultList)):
            print(f"ResultList: {resultList[i]}")
            if resultList[i].solution == []:
                vollstaendiger_pfad = False
                break

        if vollstaendiger_pfad:
            try:
                print(f"Final whole solution: {whole_solution}")

                # Visualisierung des gesamten Pfades
                # Umwandeln der Knoten-Namen in Koordinaten
                coordinates = whole_solution

                # Visualisierung des Environments
                fig, ax = plt.subplots(figsize=(10, 10))
                ax.set_title("Solution Path with Environment")

                # Zeichnen der Hindernisse im Environment
                planner._collisionChecker.drawObstacles(ax)

                # Zeichnen des Pfads der Lösung
                path_x = [coord[0] for coord in coordinates]
                path_y = [coord[1] for coord in coordinates]
                ax.plot(path_x, path_y, 'go-', label='Solution Path', zorder=2)  # 'go-' steht für grüne Punkte und Linien

                # Hervorheben des Startpunkts
                ax.plot(coordinates[0][0], coordinates[0][1], 'bo', markersize=10, label='Start', zorder=3)  # 'bo' steht für blauer Punkt

                # Hervorhebn der Zwischenziele
                for nodes in pastgoals[:-1]:
                    ax.plot(nodes[0], nodes[1], 'C6o', markersize=10, label='intermediate Goal', zorder=3)

                # Hervorheben des Zielpunkts
                ax.plot(coordinates[-1][0], coordinates[-1][1], 'ro', markersize=10, label='Goal', zorder=3)  # 'ro' steht für roter Punkt

                # Setzen der Achsenbeschriftung
                Env_Limits = planner._collisionChecker.getEnvironmentLimits()
                x_Limits = Env_Limits[0]
                y_Limits = Env_Limits[1]
                ax.set_xticks(range(int(x_Limits[0]), int(x_Limits[1]) + 1))
                ax.set_yticks(range(int(y_Limits[0]), int(y_Limits[1]) + 1))
                ax.legend()
                ax.grid(True)
                plt.show()
            except Exception as e:
                print(f"An error occurred: {e}")

        return resultList
