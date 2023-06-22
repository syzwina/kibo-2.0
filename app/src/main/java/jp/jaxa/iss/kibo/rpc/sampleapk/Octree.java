package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.ArrayList;
import java.util.List;

public class Octree {
    private final int MAX_DEPTH = 8; // Maximum recursion depth
    private final int MAX_OBJECTS = 10; // Maximum number of objects in a single node
    private final int CHILDREN_COUNT = 8; // Number of children for each node

    private OctreeNode root;

    public Octree() {
        root = new OctreeNode(0, 0, 0, 100, 100, 100); // Set the root node to cover the entire space (adjust dimensions as needed)
    }

    public void insert(Zone zone) {
        root.insert(zone);
    }

    // Implementation of OctreeNode class
    private class OctreeNode {
        private int depth;
        private final float x_min, y_min, z_min, x_max, y_max, z_max;
        private final float minSize;
        private List<Zone> zones;
        private OctreeNode[] children;

        public OctreeNode(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max, float minSize) {
            this.x_min = x_min;
            this.y_min = y_min;
            this.z_min = z_min;
            this.x_max = x_max;
            this.y_max = y_max;
            this.z_max = z_max;
            this.minSize = minSize;
            this.zones = new ArrayList<>();
            this.children = new OctreeNode[CHILDREN_COUNT];
        }

        public void insert(Zone zone) {
            if (depth < MAX_DEPTH) {
                if (children[0] == null) {
                    subdivide();
                }

                for (OctreeNode child : children) {
                    if (child.intersects(zone)) {
                        child.insert(zone);
                    }
                }
            }

            zones.add(zone);

            if (zones.size() > MAX_OBJECTS && depth < MAX_DEPTH) {
                if (children[0] == null) {
                    subdivide();
                }

                for (OctreeNode child : children) {
                    for (Zone storedZone : zones) {
                        if (child.intersects(storedZone)) {
                            child.insert(storedZone);
                        }
                    }
                }

                zones.clear();
            }
        }

        private void subdivide() {
            float x_mid = (x_min + x_max) / 2;
            float y_mid = (y_min + y_max) / 2;
            float z_mid = (z_min + z_max) / 2;

            children[0] = new OctreeNode(x_min, y_min, z_min, x_mid, y_mid, z_mid);
            children[1] = new OctreeNode(x_mid, y_min, z_min, x_max, y_mid, z_mid);
            children[2] = new OctreeNode(x_min, y_mid, z_min, x_mid, y_max, z_mid);
            children[3] = new OctreeNode(x_mid, y_mid, z_min, x_max, y_max, z_mid);
            children[4] = new OctreeNode(x_min, y_min, z_mid, x_mid, y_mid, z_max);
            children[5] = new OctreeNode(x_mid, y_min, z_mid, x_max, y_mid, z_max);
            children[6] = new OctreeNode(x_min, y_mid, z_mid, x_mid, y_max, z_max);
            children[7] = new OctreeNode(x_mid, y_mid, z_mid, x_max, y_max, z_max);

            for (OctreeNode child : children) {
                child.depth = depth + 1;
            }
        }

        private boolean intersects(Zone zone) {
            // Check if the given zone intersects with the node's bounding box
            return x_min <= zone.x_max && x_max >= zone.x_min &&
                   y_min <= zone.y_max && y_max >= zone.y_min &&
                   z_min <= zone.z_max && z_max >= zone.z_min;
        }
        
        public void divideAndAdd(float xMin, float yMin, float zMin, float xMax, float yMax, float zMax) {
            if ((yMax - yMin) <= minSize) {
                return;
            }
            
            if (children == null) {
                children = new OctreeNode[8];
            }
            
            boolean dividing = false;
            
            float xMid = (xMin + xMax) / 2;
            float yMid = (yMin + yMax) / 2;
            float zMid = (zMin + zMax) / 2;
            
            children[0] = new OctreeNode(xMin, yMin, zMin, xMid, yMid, zMid);
            children[1] = new OctreeNode(xMid, yMin, zMin, xMax, yMid, zMid);
            children[2] = new OctreeNode(xMin, yMid, zMin, xMid, yMax, zMid);
            children[3] = new OctreeNode(xMid, yMid, zMin, xMax, yMax, zMid);
            children[4] = new OctreeNode(xMin, yMin, zMid, xMid, yMid, zMax);
            children[5] = new OctreeNode(xMid, yMin, zMid, xMax, yMid, zMax);
            children[6] = new OctreeNode(xMin, yMid, zMid, xMid, yMax, zMax);
            children[7] = new OctreeNode(xMid, yMid, zMid, xMax, yMax, zMax);
            
            for (OctreeNode child : children) {
                child.depth = depth + 1;
            }
            
            for (int i = 0; i < 8; i++) {
                if ((childBounds[i].yMax - childBounds[i].yMin) > minSize) {
                    dividing = true;
                    children[i].divideAndAdd(childBounds[i].xMin, childBounds[i].yMin, childBounds[i].zMin,
                                             childBounds[i].xMax, childBounds[i].yMax, childBounds[i].zMax);
                }
            }
            
            if (!dividing) {
                children = null;
            }
        }
        

    }
}
