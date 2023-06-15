package jp.jaxa.iss.kibo.rpc.sampleapk;
import java.util.*;

import gov.nasa.arc.astrobee.types.Point;



class Node {
    public Point point;
    public double gScore;
    public double fScore;
    public Node parent;

    public Node(Point point, double gScore, double fScore, Node parent) {
        this.point = point;
        this.gScore = gScore;
        this.fScore = fScore;
        this.parent = parent;
    }
}

public class Pathfinding {

    // initializing zones
    private final KeepOutZone KOZ01 = new KeepOutZone(10.783f, -9.8899f, 4.8385f, 11.071f, -9.6929f, 5.0665f);
    private final KeepOutZone KOZ02 = new KeepOutZone(10.8652f, -9.0734f, 4.3861f, 10.9628f, -8.7314f, 4.6401f);
    private final KeepOutZone KOZ03 = new KeepOutZone(10.185f, -8.3826f, 4.1475f, 11.665f, -8.2826f, 4.6725f);
    private final KeepOutZone KOZ04 = new KeepOutZone(10.7955f, -8.0635f, 5.1055f, 11.3525f, -7.7305f, 5.1305f);
    private final KeepOutZone KOZ05 = new KeepOutZone(10.563f, -7.1449f, 4.6544f, 10.709f, -6.8099f, 4.8164f);
    List<KeepOutZone> ZONES = Arrays.asList(KOZ01,KOZ02,KOZ03,KOZ04,KOZ05);
    private KeepOutZone currentKOZ = new KeepOutZone(0f,0f,0f,0f,0f,0f); // not really needed but put it here anyway

    private List<Point> findPath(Point start, Point goal, List<KeepOutZone> zones) {
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(node -> node.fscore));
        Map<Point, Node> nodeMap = new HashMap<>();
        openSet.add(new Node(start, 0, calculateHeuristic(start, goal), null));
        nodeMap.put(start, openSet.peek());

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();
            if (current.point.equals(goal)) {
                return reconstructPath(current);
            }

            for (Point neighbor : getNeighbors(current.point)) {
                double tentativeGScore = current.gScore + calculateDistance(current.point, neighbor);
                if (isInKeepOutZone(neighbor, zones) || tentativeGScore >= getGScore(nodeMap, neighbor))
                    continue;

                Node neighborNode = nodeMap.get(neighbor);
                if (neighborNode == null) {
                    neighborNode = new Node(neighbor, tentativeGScore, calculateHeuristic(neighbor, goal), current);
                    nodeMap.put(neighbor, neighborNode);
                    openSet.add(neighborNode);
                } else {
                    neighborNode.parent = current;
                    neighborNode.gScore = tentativeGScore;
                    neighborNode.fScore = neighborNode.gScore + calculateHeuristic(neighbor, goal);
                }
            }
        }

        return Collections.emptyList(); // No path found
    }

    private double calculateHeuristic(Point start, Point goal) {
        // Implement your heuristic calculation here, e.g., Euclidean distance
        return Math.sqrt(Math.pow(start.getX() - goal.getX(), 2) + Math.pow(start.getY() - goal.getY(), 2) + Math.pow(start.getZ() - goal.getZ(), 2));
    }

    private double calculateDistance(Point a, Point b) {
        // Implement your distance calculation here, e.g., Euclidean distance
        return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2) + Math.pow(a.getZ() - b.getZ(), 2));
    }

    private boolean isInKeepOutZone(Point point, List<KeepOutZone> zones) {
        // Implement your check to determine if a point is within a keepout zone
        for (KeepOutZone zone : ZONES) {
            if (point.getX() >= zone.x_min && point.getX() <= zone.x_max &&
                    point.getY() >= zone.y_min && point.getY() <= zone.y_max &&
                    point.getZ() >= zone.z_min && point.getZ() <= zone.z_max) {
                return true;
            }
        }
        return false;
    }

    private List<Point> getNeighbors(Point point) {
        // Implement getting neighboring points based on your environment
        // e.g., consider 6 or 26 connected points
        List<Point> neighbors = new ArrayList<>();
        // Add neighboring points to the 'neighbors' list
        return neighbors;
    }

    private double getGScore(Map<Point, Node> nodeMap, Point point) {
        Node node = nodeMap.get(point);
        return node != null ? node.gScore : Double.MAX_VALUE;
    }

    private List<Point> reconstructPath(Node goalNode) {
        List<Point> path = new ArrayList<>();
        Node current = goalNode;
        while (current != null) {
            path.add(0, current.point);
            current = current.parent;
        }
        return path;
    }
}

