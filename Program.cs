using System;
using System.Collections.Generic;
using System.Linq;

/**
 * Author: Yanzhi Wang
 * 
 * This class provides an implementation of Dijkstra's algorithm for finding the shortest path 
 * between two nodes in a graph. The implementation uses a priority queue to efficiently find 
 * the shortest path from the start node to each of the other nodes in the graph.
 * 
 * Restrictions on the usage of this class include:
 * - The nodes in the graph must be represented as instances of the Node class defined in this file.
 * - The edges between nodes must be represented as instances of the Edge class defined in this file.
 * - The graph must not contain any negative edge weights, as this will cause the algorithm to fail.
 * 
 * Known errors or issues with this class include:
 * - None known at this time.
 */

public static class Dijkstra

{
    public enum EColor
    {
        red,
        blue,
        yellow,
        cyan,
        gray,
        purple,
        orange,
        green
    }
    // List of all nodes
    public static List<Node> colorNodes = new List<Node>();

    // Node class
    public class Node : IComparable<Node>
    {
        public EColor nState;

        public List<Edge> edges = new List<Edge>();

        public int minCostToStart;
        public Node nearestToStart;
        public bool visited;

        public Node(EColor nState)
        {
            this.nState = nState;
            this.minCostToStart = int.MaxValue;
        }

        public void AddEdge(int cost, Node connection)
        {
            Edge e = new Edge(cost, connection);
            edges.Add(e);
        }

        public int CompareTo(Node n)
        {
            return this.minCostToStart.CompareTo(n.minCostToStart);
        }
    }

    // Edge class
    public class Edge : IComparable<Edge>
    {
        public int cost;
        public Node connectedNode;

        public Edge(int cost, Node connectedNode)
        {
            this.cost = cost;
            this.connectedNode = connectedNode;
        }

        public int CompareTo(Edge e)
        {
            return this.cost.CompareTo(e.cost);
        }
    }

    // Dijkstra's algorithm to find the shortest path
    static public List<Node> GetShortestPathDijkstra()
    {
        DijkstraSearch();
        List<Node> shortestPath = new List<Node>();
        shortestPath.Add(colorNodes[7]);
        BuildShortestPath(shortestPath, colorNodes[7]);
        shortestPath.Reverse();
        return (shortestPath);
    }

    static private void BuildShortestPath(List<Node> list, Node node)
    {
        if (node.nearestToStart == null)
        {
            return;
        }
        list.Add(node.nearestToStart);
        BuildShortestPath(list, node.nearestToStart);
    }

    static private int NodeOrderBy(Node n)
    {
        return n.minCostToStart;
    }

    static private void DijkstraSearch()
    {
        Node start = colorNodes[0];

        start.minCostToStart = 0;
        List<Node> prioQueue = new List<Node>();
        prioQueue.Add(start);

        Func<Node, int> nodeOrderBy = NodeOrderBy;

        do
        {
            prioQueue.Sort();

            Node node = prioQueue.First();
            prioQueue.Remove(node);
            foreach (Edge cnn in
                node.edges.OrderBy(delegate (Edge n) { return n.cost; }))
            {
                Node childNode = cnn.connectedNode;
                if (childNode.visited)
                {
                    continue;
                }

                if (childNode.minCostToStart == int.MaxValue ||
                    node.minCostToStart + cnn.cost < childNode.minCostToStart)
                {
                    childNode.minCostToStart = node.minCostToStart + cnn.cost;
                    childNode.nearestToStart = node;
                    if (!prioQueue.Contains(childNode))
                    {
                        prioQueue.Add(childNode);
                    }
                }
            }
            node.visited = true;

            if (node == colorNodes[7])
            {
                return;
            }
        } while (prioQueue.Any());

    }
}

class Program
{
    static void Main(string[] args)
    {
        // Create nodes
        Dijkstra.Node node1 = new Dijkstra.Node(Dijkstra.EColor.red);
        Dijkstra.Node node2 = new Dijkstra.Node(Dijkstra.EColor.blue);
        Dijkstra.Node node3 = new Dijkstra.Node(Dijkstra.EColor.yellow);
        Dijkstra.Node node4 = new Dijkstra.Node(Dijkstra.EColor.cyan);
        Dijkstra.Node node5 = new Dijkstra.Node(Dijkstra.EColor.gray);
        Dijkstra.Node node6 = new Dijkstra.Node(Dijkstra.EColor.purple);
        Dijkstra.Node node7 = new Dijkstra.Node(Dijkstra.EColor.orange);
        Dijkstra.Node node8 = new Dijkstra.Node(Dijkstra.EColor.green);
        // Create edges between nodes
        node1.AddEdge(2, node2);
        node1.AddEdge(4, node3);
        node2.AddEdge(3, node4);
        node2.AddEdge(2, node5);
        node2.AddEdge(1, node3);
        node3.AddEdge(1, node6);
        node3.AddEdge(4, node7);
        node4.AddEdge(3, node8);
        node5.AddEdge(2, node8);
        node6.AddEdge(1, node8);
        node7.AddEdge(4, node8);

        // Add nodes to colorNodes list
        Dijkstra.colorNodes.Add(node1);
        Dijkstra.colorNodes.Add(node2);
        Dijkstra.colorNodes.Add(node3);
        Dijkstra.colorNodes.Add(node4);
        Dijkstra.colorNodes.Add(node5);
        Dijkstra.colorNodes.Add(node6);
        Dijkstra.colorNodes.Add(node7);
        Dijkstra.colorNodes.Add(node8);

        // Find shortest path using Dijkstra's algorithm
        List<Dijkstra.Node> shortestPath = Dijkstra.GetShortestPathDijkstra();

        // Output the shortest path
        Console.WriteLine("Shortest Path: ");
        foreach (Dijkstra.Node node in shortestPath)
        {
            Console.Write(node.nState + " ");
        }
        Console.ReadLine();
    }
}
