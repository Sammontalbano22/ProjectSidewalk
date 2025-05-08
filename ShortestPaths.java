//Barrett Altman, Sam Montalbano
//3/16/2025
//Primary Code for the Project
package graph;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;

import java.util.LinkedList;
import java.util.PriorityQueue;

import org.w3c.dom.Node;

import java.util.HashMap;
import java.util.LinkedList;
import java.io.File;
import java.io.FileNotFoundException;

/** Provides an implementation of Dijkstra's single-source shortest paths
 * algorithm.
 * Sample usage:
 *   Graph g = // create your graph
 *   ShortestPaths sp = new ShortestPaths();
 *   Node a = g.getNode("A");
 *   sp.compute(a);
 *   Node b = g.getNode("B");
 *   LinkedList<Node> abPath = sp.getShortestPath(b);
 *   double abPathLength = sp.getShortestPathLength(b);
 *   */
private static class NodeDistance implements Comparable<NodeDistance> {
    Node node;
    double distance;

    public NodeDistance(Node node, double distance) {
        this.node = node;
        this.distance = distance;
    }

    @Override
    public int compareTo(NodeDistance other) {
        return Double.compare(this.distance, other.distance);
    }
}

public class ShortestPaths {
    // stores auxiliary data associated with each node for the shortest
    // paths computation:
    private HashMap<Node,PathData> paths;
    /** Compute the shortest path to all nodes from origin using Dijkstra's
     * algorithm. Fill in the paths field, which associates each Node with its
     * PathData record, storing total distance from the source, and the
     * back pointer to the previous node on the shortest path.
     * Precondition: origin is a node in the Graph.*/
    // TODO 1: implement Dijkstra's algorithm to fill paths with
    public void compute(Node origin) {
        paths = new HashMap<>(); //Instantiates paths hashMap
        paths.put(origin, new PathData(0.0, null)); //Creates origin node within the hashMap
        PriorityQueue<Node> frontier = new PriorityQueue<>((a, b) -> Double.compare(paths.get(a).distance, paths.get(b).distance)); //Establishes frontier to take two nodes as keys, and returns shorest distance between two nodes.
        frontier.add(origin);

        while (!frontier.isEmpty()) {
            Node currentNode = frontier.poll(); //takes node at front
            double currentDistance = paths.get(currentNode).distance; //take distance from origin to current node.

            for (Map.Entry<Node, Double> edge : currentNode.getNeighbors().entrySet()) { //Relaxation step, take weight and distance to find shortest path amongst analyzed nodes.
                Node neighbor = edge.getKey();//O(1)
                double edgeWeight = edge.getValue();//O(1)
                double newDistance = currentDistance + edgeWeight; //O(1)

                if (!paths.containsKey(neighbor) || newDistance < paths.get(neighbor).distance) { //O(log n) If it hasn't yet seen node, or current path is shorter, adds to paths and frontier.
                    paths.put(neighbor, new PathData(newDistance, currentNode));
                    frontier.add(neighbor); //O(log n)
                }
            }
        }
    }


        // shortest-path data for each Node reachable from origin.
//       * Compute shortest paths to all nodes from origin nodes */
//        shortest_paths(v);
//        S = {};
//        F = (v);
//        v.d = 0;
//        v.bp = null;
//        while (F != {}) {
//        f = node in F with min d value;
//        Remove f from F, add it so S;
//        for each neighbor w of f {
//        if (w not in S or F) {
//        w.d = f.d + weight(f, w);
//        w.bp = f;
//        add w to F;
//        } else if (f.d + weight(f, w) < w.d) {
//        w.d = f.d + weight(f, w);
//        w.bp = f;
//        }
//        }
//        }

    /** Returns the length of the shortest path from the origin to destination.
     * If no path exists, return Double.POSITIVE_INFINITY.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called. */
    public double shortestPathLength(Node destination) {
        Node current = destination;
        while (pathes.get(destination).previous != null){
            current = pathes.get(destination).previous; //Goes through nodes until the previous node is null. Checks if that node is the destination, if not, returns arbitrarily large number.
        }
        if (current != destination){
            return Double.POSITIVE_INFINITY
        }
        else{
            return pathes.get(destination).length
        }

    }

    /** Returns a LinkedList of the nodes along the shortest path from origin
     * to destination. This path includes the origin and destination. If origin
     * and destination are the same node, it is included only once.
     * If no path to it exists, return null.
     * Precondition: destination is a node in the graph, and compute(origin)
     * has been called. */
    public LinkedList<Node> shortestPath(Node destination) {
        // TODO 3 - implement this method to reconstruct sequence of Nodes
        // along the shortest path from the origin to destination using the
        // paths data computed by Dijkstra's algorithm.
        LinkedList<Node> path = new LinkedList<Node>();
        Node current = destination;
        while (current != null) {
            path.addFirst(current);
            current = paths.get(current).previous;
        }
        return path;
    }


    /** Inner class representing data used by Dijkstra's algorithm in the
     * process of computing shortest paths from a given source node. */
    class PathData {
        double distance; // distance of the shortest path from source
        Node previous; // previous node in the path from the source

        /** constructor: initialize distance and previous node */
        public PathData(double dist, Node prev, Node current) {
            distance = dist;
            previous = prev;
        }
    }


    /** Static helper method to open and parse a file containing graph
     * information. Can parse either a basic file or a CSV file with
     * sidewalk data. See GraphParser, BasicParser, and DBParser for more.*/
    protected static Graph parseGraph(String fileType, String fileName) throws
        FileNotFoundException {
        // create an appropriate parser for the given file type
        GraphParser parser;
        if (fileType.equals("basic")) {
            parser = new BasicParser();
        } else if (fileType.equals("db")) {
            parser = new DBParser();
        } else {
            throw new IllegalArgumentException(
                    "Unsupported file type: " + fileType);
        }

        // open the given file
        parser.open(new File(fileName));

        // parse the file and return the graph
        return parser.parse();
    }

    public static void main(String[] args) {
        // read command line args
        String fileType = args[0];
        String fileName = args[1];
        String SidewalkOrigCode = args[2];

        String SidewalkDestCode = null;
        if (args.length == 4) {
            SidewalkDestCode = args[3];
        }

        // parse a graph with the given type and filename
        Graph graph;
        try {
            graph = parseGraph(fileType, fileName);
        } catch (FileNotFoundException e) {
            System.out.println("Could not open file " + fileName);
            return;
        }
        graph.report();


        // TODO 4: create a ShortestPaths object, use it to compute shortest
        // paths data from the origin node given by origCode.
        ShortestPaths sPath = new ShortestPaths();
        sPath.compute(SideWalkOrigCode);

        

        // TODO 5:
        // If destCode was not given, print each reachable node followed by the
        // length of the shortest path to it from the origin.
        if (SidewalkDestCode == null){
            Set<Node> keys = sPath.pathes.keySet();
            for (Node element : keys){
                System.out.println(element, sPath.pathes.get(element).distance)
            }
        }

        // TODO 6:
        // If destCode was given, print the nodes in the path from
        // origCode to destCode, followed by the total path length
        // If no path exists, print a message saying so.

        else if (SidewalkDestCode != null){
            LinkedList<Node> nodePath = sPath.ShortestPath(SideWalkDestCode);
            int pathLength = sPath.shortestPathLength(SideWalkDestCode);
            for (int i = 0; i<nodePath.length(); i++){
                System.out.println(nodePath.get(i), sPath.shortestPathLength(i));
            }
        }


        }
}
