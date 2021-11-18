#include <iostream>
#include <sstream>
#include <istream>
#include <vector>
#include <deque>
#include <set>
#include <algorithm>
#include <utility>
#include <cmath>
#include <limits>
#include <memory>
#include <lemon/list_graph.h>
#include <lemon/dfs.h>
#include <lemon/bfs.h>
#include <lemon/dijkstra.h>

#define MAX_INTERMEDIATE_PATH_LENGTH 4

using namespace std;
using namespace lemon;


class SMTGraph {
    private:
        istream& is;
        ListDigraph& graph;
        ListDigraph::ArcMap<int> edgeDistances;

        int numberOfVertices;
        int numberOfEdges;
        ListDigraph::Node sourceNode;
        ListDigraph::Node destinationNode;

        vector<vector<pair<int, int>>> edgesLeaving;
        vector<vector<pair<int, int>>> edgesArriving;

        void readGraphProperties() {
            int pathOrigin, pathDestination;
            string line;
            getline(is, line);
            istringstream iss(line);
            iss >> numberOfVertices >> numberOfEdges >> pathOrigin >> pathDestination;
            
            for(int i = 0; i < numberOfVertices; i++) {
                graph.addNode();
            }

            sourceNode = graph.nodeFromId(pathOrigin-1);
            destinationNode = graph.nodeFromId(pathDestination-1);
        }

        void readEdges() {
            string line;
            for (int i = 0; i < numberOfEdges; i++) {
                int edgeDistance;
                int u, v;
                getline(is, line);
                istringstream iss(line);
                iss >> u >> v >> edgeDistance >> ws;
                
                ListDigraph::Arc edge = graph.addArc(graph.nodeFromId(u-1), graph.nodeFromId(v-1));
                ListDigraph::Arc reverseEdge = graph.addArc(graph.nodeFromId(v-1), graph.nodeFromId(u-1));
                edgeDistances[edge] = edgeDistance;
                edgeDistances[reverseEdge] = edgeDistance;
            }
        }

    public:

        SMTGraph(istream& is, ListDigraph& graph)
         : is(is), graph(graph), edgeDistances(graph) {
            readGraphProperties();
            readEdges();
        }
        ~SMTGraph() {
        }
        

        ListDigraph& getGraph() { return graph; }
        ListDigraph::Node getSource() { return sourceNode; }
        ListDigraph::Node getDestination() { return destinationNode; }

        int getDistanceOfArc(const ListDigraph::Arc& arc) { return edgeDistances[arc]; }

        ListPath<ListDigraph> shortestPath() {
            //Dijkstra<ListDigraph> dijkstra(graph, edgeDistances);
            Bfs<ListDigraph> dijkstra(graph);
            dijkstra.init();
            dijkstra.run(sourceNode, destinationNode);

            return dijkstra.path(destinationNode);
        }
};


class tabuSearchInstance { 
    public:
        virtual float fitness() = 0;
        virtual list<shared_ptr<tabuSearchInstance>> neighborhood() = 0;
};


bool comparePaths(const ListDigraph& g, const ListPath<ListDigraph>& path, const ListPath<ListDigraph>& otherPath) {
    if (path.length() != otherPath.length()) {
        return false;
    }
    PathNodeIt<ListPath<ListDigraph>> pathNodeIt(g, path);
    PathNodeIt<ListPath<ListDigraph>> otherPathNodeIt(g, otherPath);
    while (pathNodeIt != INVALID) {
        ListDigraph::Node pathNode = pathNodeIt;
        ListDigraph::Node otherPathNode = otherPathNodeIt;
        if (pathNode != otherPathNode) {
            return false;
        }
        ++pathNodeIt;
        ++otherPathNodeIt;
    }
    return true;
}


class SMTInstance : public tabuSearchInstance {
    private:
        SMTGraph& graph;
        ListPath<ListDigraph> path;
        int maxStep;


        void createInitialPath() {
            path = graph.shortestPath();
            printPath();
        }

        void printPath() {
            PathNodeIt<ListPath<ListDigraph>> node(graph.getGraph(), path);
            std::cout << graph.getGraph().id(node)+1;
            ++node;
            for ( ; node != INVALID; ++node) {  
                std::cout << " -> " << graph.getGraph().id(node)+1;
            }
            std::cout << std::endl;
        }
        
        void removeInArcsOf(ListDigraph& g, const ListDigraph::Node& node) {
            for (ListDigraph::InArcIt inArc(g, node); inArc != INVALID;) {
                ListDigraph::Arc arcToRemove = inArc;
                ++inArc;
                g.erase(arcToRemove);
            }
        }

        void removeOutArcsOf(ListDigraph& g, const ListDigraph::Node& node) {
            for (ListDigraph::OutArcIt outArc(g, node); outArc != INVALID;) {
                ListDigraph::Arc arcToRemove = outArc;
                ++outArc;
                g.erase(arcToRemove);
            }
        }
        
        void addOutArcsBack(ListDigraph& g, const ListDigraph::Node& node) {
            for (ListDigraph::InArcIt inArc(g, node); inArc != INVALID; ++inArc) {
                ListDigraph::Node reverseArcSource = g.target(inArc);
                ListDigraph::Node reverseArcTarget = g.source(inArc);
                g.addArc(reverseArcSource, reverseArcTarget);
            }
        }

        void addInArcsBack(ListDigraph& g, const ListDigraph::Node& node) {
            for (ListDigraph::OutArcIt outArc(g, node); outArc != INVALID; ++outArc) {
                ListDigraph::Node reverseArcSource = g.target(outArc);
                ListDigraph::Node reverseArcTarget = g.source(outArc);
                g.addArc(reverseArcSource, reverseArcTarget);
            }
        }

        std::vector<ListPath<ListDigraph>> findAllPathsBetween(const ListDigraph::Node& source, const ListDigraph::Node& destination) {
            ListDigraph g;
            ListDigraph::Node graphSrc;
            ListDigraph::Node graphDst;
            ListDigraph::Node src;
            ListDigraph::Node dst;
            CrossRefMap<ListDigraph, ListDigraph::Node, ListDigraph::Node> originalNode(graph.getGraph());
            digraphCopy(graph.getGraph(), g).node(graph.getSource(), graphSrc)
                                            .node(graph.getDestination(), graphDst)
                                            .node(source, src)
                                            .node(destination, dst)
                                            .nodeCrossRef(originalNode).run();

            const size_t maxPathLength = MAX_INTERMEDIATE_PATH_LENGTH;
            
            std::vector<ListPath<ListDigraph>> paths;
            
            removeInArcsOf(g, graphDst);
            removeOutArcsOf(g, graphSrc);
            removeInArcsOf(g, dst);
            removeOutArcsOf(g, src);

            
            struct TreeQueueNode {
                ListDigraph::Node node;
                int level;
            };
            std::list<TreeQueueNode> queue;

            ListDigraph pathTree;
            ListDigraph::NodeMap<ListDigraph::Node> treeNodeToGraphNode(pathTree);

            ListDigraph::Node rootNode = pathTree.addNode();
            queue.push_back({rootNode, 0});
            treeNodeToGraphNode[rootNode] = dst;

            while (!queue.empty()) { 
                TreeQueueNode parent = queue.front();
                if (parent.level < maxPathLength) {
                    for (ListDigraph::OutArcIt outArc(g, treeNodeToGraphNode[parent.node]); 
                            outArc != INVALID; ++outArc) {
                        ListDigraph::Node newNode = pathTree.addNode();
                        treeNodeToGraphNode[newNode] = g.runningNode(outArc);
                        pathTree.addArc(newNode, parent.node);

                        bool hasCycles = false;
                        ListDigraph::OutArcIt initialParentArc(pathTree, newNode);
                        ListDigraph::Node parentNode = pathTree.runningNode(initialParentArc);
                        while(parentNode != INVALID) {
                            if(treeNodeToGraphNode[newNode] == treeNodeToGraphNode[parentNode]) {
                                hasCycles = true;
                            }
                            ListDigraph::OutArcIt parentArc(pathTree, parentNode);
                            parentNode = (parentArc != INVALID) ? pathTree.runningNode(parentArc)
                                                                : INVALID;
                        }

                        if(!hasCycles) {
                            queue.push_back({newNode, parent.level+1});
                        }
                    }
                }
                queue.pop_front();
            }

            addOutArcsBack(g, src);
            addInArcsBack(g, dst);
            addOutArcsBack(g, graphSrc);
            addInArcsBack(g, graphDst);

            Dfs<ListDigraph> dfs(pathTree);
            dfs.init();

            for (ListDigraph::NodeIt treeNode(pathTree); treeNode != INVALID; ++treeNode) {
                if (treeNodeToGraphNode[treeNode] == src) {
                    dfs.run(treeNode);
                    ListPath<ListDigraph> treePath = dfs.path(rootNode);

                    PathNodeIt<ListPath<ListDigraph>> treePathNodeU(pathTree, treePath);
                    PathNodeIt<ListPath<ListDigraph>> treePathNodeV(pathTree, treePath);
                    ++treePathNodeV;
                    ListPath<ListDigraph> graphPath;
                    while (treePathNodeV != INVALID) {
                        ListDigraph::Node u = originalNode[treeNodeToGraphNode[treePathNodeU]];
                        ListDigraph::Node v = originalNode[treeNodeToGraphNode[treePathNodeV]];
                        ListDigraph::Arc graphPathArc = findArc(graph.getGraph(), u, v);
                        graphPath.addBack(graphPathArc);
                        
                        ++treePathNodeU;
                        ++treePathNodeV;
                    }
                    paths.push_back(graphPath);
                }
            }

            return paths;
        } 

        vector<ListPath<ListDigraph>> generateNeighborPaths() {
            vector<ListPath<ListDigraph>> neighborPaths;
            vector<ListPath<ListDigraph>> intermediatePaths;
            int pathLength = path.length();

            ListPath<ListDigraph> pathBegin = path;
            ListPath<ListDigraph> pathMiddle;
            ListPath<ListDigraph> pathEnd;
            
            int cutStartIndex = rand() % (pathLength - 1);
            ListPath<ListDigraph>::ArcIt cutStartArc = pathBegin.nthIt(cutStartIndex);
            ListPath<ListDigraph>::ArcIt cutEndArc = pathBegin.nthIt(cutStartIndex + 1);

            ListDigraph::Node cutEndNode = graph.getGraph().target(cutEndArc);
            ++cutEndArc;
            pathBegin.split(cutEndArc, pathEnd);

            ListDigraph::Node cutStartNode = graph.getGraph().source(cutStartArc);
            pathBegin.split(cutStartArc, pathMiddle);

            intermediatePaths = findAllPathsBetween(cutStartNode, cutEndNode);

            for (auto intermediatePath : intermediatePaths) {
                ListPath<ListDigraph> neighborPath = pathBegin;
                ListPath<ListDigraph> neighborEnd = pathEnd;
                
                neighborPath.spliceBack(intermediatePath);
                neighborPath.spliceBack(neighborEnd);

                if (!comparePaths(graph.getGraph(), path, neighborPath)) {
                    neighborPaths.push_back(neighborPath);
                }
            }

//            for (auto path : neighborPaths) {
//                for (PathNodeIt<ListPath<ListDigraph>> node(graph.getGraph(), path); node != INVALID; ++node) {
//                    std::cout << "Neighbor: " << graph.getGraph().id(node) << std::endl;
//                }
//                std::cout << std::endl;                                                            
//            }                                                                                      
            
            return neighborPaths;
        }

    public:
        SMTInstance(SMTGraph& graph)
         : graph(graph), maxStep(-1) {
            createInitialPath();
        }

        SMTInstance(SMTGraph& graph, ListPath<ListDigraph> path) 
         : graph(graph), path(path), maxStep(-1) {
             //printPath();
             fitness();
        }

        ~SMTInstance() {
        }

        float fitness() override {
            if (maxStep < 0) {
                maxStep = 0;
                ListPath<ListDigraph>::ArcIt previousArc(path);
                ListPath<ListDigraph>::ArcIt nextArc = previousArc;
                ++nextArc;
                while (nextArc != INVALID) {
                    int step = abs(graph.getDistanceOfArc(previousArc) - graph.getDistanceOfArc(nextArc));
                    if (step > maxStep) {
                        maxStep = step;
                    }
                    ++previousArc;
                    ++nextArc;
                }
            }
            return maxStep;
        }

        list<shared_ptr<tabuSearchInstance>> neighborhood() override {
            list<shared_ptr<tabuSearchInstance>> neighbors;
            vector<ListPath<ListDigraph>> neighborPaths = generateNeighborPaths();
            for (auto path : neighborPaths) {
                shared_ptr<SMTInstance> neighbor =  make_shared<SMTInstance>(graph, path);
                neighbors.push_back(neighbor);
            }
            return neighbors;
        }

        bool operator==(const SMTInstance& other) {
            return comparePaths(graph.getGraph(), this->path, other.path);
        }
};


template<typename Iter>
bool instanceInList(const shared_ptr<tabuSearchInstance>& instance, Iter begin, Iter end) {
    return find_if(begin, end, 
                   [&instance](shared_ptr<tabuSearchInstance>& listElement) {
                       return *((SMTInstance*) listElement.get()) 
                              == *((SMTInstance*) instance.get());
                   }) != end;
}

bool instanceInList(const shared_ptr<tabuSearchInstance>& instance, 
                    const deque<shared_ptr<tabuSearchInstance>>& list) {
    for (auto element : list) {
        if (*((SMTInstance*) element.get()) == *((SMTInstance*) instance.get())) {
            return true;
        }
    }
    return false;
}


float tabuSearch(const shared_ptr<tabuSearchInstance> initialState, float timeLimit) {
    const int maxTabuSize = 100;
    shared_ptr<tabuSearchInstance> sBest = initialState;
    shared_ptr<tabuSearchInstance> bestCandidate = initialState;
    deque<shared_ptr<tabuSearchInstance>> tabuList;
    tabuList.push_back(initialState);

    int iteration = 0;
    time_t beginTime = time(NULL);
    float runTime = 0.0;
    while (runTime < timeLimit && !tabuList.empty()) {
        list<shared_ptr<tabuSearchInstance>> neighborhood = bestCandidate->neighborhood();
        for (auto candidateIt = neighborhood.begin(); candidateIt != neighborhood.end(); ) {
            if (instanceInList(*candidateIt, tabuList)) {
                auto toErase = candidateIt;
                candidateIt++;
                neighborhood.erase(toErase);
            } else {
                candidateIt++;
            }
        }

        if(neighborhood.empty()) {
            break;
        }
        bestCandidate = neighborhood.front();
        neighborhood.pop_front();
        for (auto candidate : neighborhood) {
            if ( candidate->fitness() < bestCandidate->fitness() ) {
                bestCandidate = candidate;
            }
        }
        //cout << "Best of iteration " << iteration << ": " << bestCandidate->fitness() << endl;
        if (bestCandidate->fitness() < sBest->fitness()) {
            sBest = bestCandidate;
        }
        tabuList.push_front(bestCandidate);
        if (tabuList.size() > maxTabuSize) {
            tabuList.pop_back();
        }
        
        if (iteration % 250 == 0) {
            cout << "Best result on iteration " << iteration << ": " << sBest->fitness() << endl;
        }

        runTime = difftime(time(NULL), beginTime);
        iteration++;
    }
    runTime = difftime(time(NULL), beginTime);
    cout << "Run time: " << runTime << endl;
    return sBest->fitness();
}


int main(int argc, char* argv[]) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " time_limit" << endl;
        cerr << endl;
        cerr << "Parameters:" << endl;
        cerr << "\ttime_limit\ttime limit in seconds" << endl;
        exit(-1);
    }

    float timeLimit = atof(argv[1]);
    
    srand(time(NULL));
    ListDigraph graph;
    SMTGraph smtGraph(cin, graph);
    shared_ptr<SMTInstance> instance = make_shared<SMTInstance>(smtGraph);
    float result = tabuSearch(instance, timeLimit);
    cout << "Best result: " << result << endl;
}
