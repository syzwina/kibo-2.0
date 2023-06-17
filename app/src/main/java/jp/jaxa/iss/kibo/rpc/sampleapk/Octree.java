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
        private float x_min, y_min, z_min, x_max, y_max, z_max;
        private List<Zone> zones;
        private OctreeNode[] children;

        public OctreeNode(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max) {
            this.x_min = x_min;
            this.y_min = y_min;
            this.z_min = z_min;
            this.x_max = x_max;
            this.y_max = y_max;
            this.z_max = z_max;
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
        
    }
}
