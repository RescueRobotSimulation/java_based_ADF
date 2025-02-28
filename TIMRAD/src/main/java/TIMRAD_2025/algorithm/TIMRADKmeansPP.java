package TIMRAD_2025.algorithm;

import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.StaticClustering;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

public class TIMRADKmeansPP extends StaticClustering {
    private static final String KEY_CLUSTER_CENTERX = "RIO2023.clustering.centerX";
    private static final String KEY_CLUSTER_CENTERY = "RIO2023.clustering.centerY";
    private static final String KEY_CLUSTER_ENTITY = "RIO2023.clustering.entities";

    private static final int REPEATS_PRECOMPUTE = 20;
    private static final int REPEATS_PREPARE = 10;

    private static final int SEED = 19970824;

    private int clusterSize;

    private StandardEntityURN myurn;

    private Collection<StandardEntity> entities;
    private ArrayList<Cluster> clusters;
    private Map<EntityID, Integer> assigns = new HashMap<>();

    public TIMRADKmeansPP(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        this.myurn = ai.me().getStandardURN();
        this.entities = worldInfo.getEntitiesOfType(
                StandardEntityURN.ROAD,
                StandardEntityURN.HYDRANT,
                StandardEntityURN.BUILDING,
                StandardEntityURN.REFUGE,
                StandardEntityURN.GAS_STATION,
                StandardEntityURN.AMBULANCE_CENTRE,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.POLICE_OFFICE
        );

        clusterSize = calcClusterSize();
    }

    private int calcClusterSize(){
        int size = worldInfo.getEntitiesOfType(myurn).size()/4;

        if(size < 1) size = 1;

        return size;
    }

    @Override
    public Clustering precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if (this.getCountPrecompute() > 1) {
            return this;
        }

        execute(REPEATS_PRECOMPUTE);

        //write
        for (int index = 0; index < this.getClusterNumber(); index++) {
            final double x = this.getClusterCenterX(index);
            final double y = this.getClusterCenterY(index);
            final List<StandardEntity> elements = this.clusters.get(index).getElements();

            List<EntityID> ids = new ArrayList<>();

            for (StandardEntity entity : elements) {
                ids.add(entity.getID());
            }

            precomputeData.setEntityIDList(KEY_CLUSTER_ENTITY + "." + this.myurn + "." + index, ids);
            precomputeData.setDouble(KEY_CLUSTER_CENTERX + "." + this.myurn + "." + index, x);
            precomputeData.setDouble(KEY_CLUSTER_CENTERY + "." + this.myurn + "." + index, y);
        }
        return this;
    }

    @Override
    public Clustering resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountResume() > 1) {
            return this;
        }

        // read
        this.clusters = new ArrayList<>();
        for (int index = 0; index < clusterSize; index++) {
            final List<EntityID> ids = precomputeData.getEntityIDList(KEY_CLUSTER_ENTITY + "." + this.myurn + "." + index);

            List<StandardEntity> elements = new ArrayList<>();
            for (EntityID id : ids) {
                elements.add(this.worldInfo.getEntity(id));
            }

            double x = precomputeData.getDouble(KEY_CLUSTER_CENTERX  + "." + this.myurn + "." + index);
            double y = precomputeData.getDouble(KEY_CLUSTER_CENTERY  + "." + this.myurn + "." + index);

            this.clusters.add(new Cluster(x, y, elements));
        }

        assignAgents(new ArrayList<>(worldInfo.getEntitiesOfType(myurn)));

        return this;
    }

    @Override
    public Clustering preparate() {
        super.preparate();
        if (this.getCountPreparate() > 1) return this;

        execute(REPEATS_PREPARE);

        assignAgents(new ArrayList<>(worldInfo.getEntitiesOfType(myurn)));
        return this;
    }

    @Override
    public int getClusterNumber() {
        return clusters.size();
    }

    @Override
    public int getClusterIndex(StandardEntity standardEntity) {
        return this.getClusterIndex(standardEntity.getID());
    }

    @Override
    public int getClusterIndex(EntityID entityID) {
        if (!this.assigns.containsKey(entityID)) {
            for (int i = 0; i < this.getClusterNumber(); i++) {
                StandardEntity entity = worldInfo.getEntity(entityID);
                if (this.clusters.get(i).getElements().contains(entity)) {
                    return i;
                }
            }
            return -1;
        }

        return this.assigns.get(entityID);
    }

    @Override
    public Collection<StandardEntity> getClusterEntities(int i) {
        if (i < 0 || i >= this.getClusterNumber()) {
            return null;
        }

        return this.clusters.get(i).getElements();
    }

    @Override
    public Collection<EntityID> getClusterEntityIDs(int i) {
        final Collection<StandardEntity> entities = this.getClusterEntities(i);
        if (entities == null) {
            return null;
        }

        Collection<EntityID> result = new ArrayList<>();
        for (StandardEntity entity : entities) {
            if (entity == null) {
                continue;
            }
            result.add(entity.getID());
        }

        return result;
    }

    public double getClusterCenterX(int i) {
        return clusters.get(i).getCenterX();
    }


    public double getClusterCenterY(int i) {
        return clusters.get(i).getCenterY();
    }

    @Override
    public Clustering calc() {
        if (!this.assigns.isEmpty()) return this;

        final int time = this.agentInfo.getTime();
        if (time < 1) return this;

        this.assignAgents(new ArrayList<>(worldInfo.getEntitiesOfType(myurn)));

        return this;
    }

    public void execute(int repeat) {
        this.clusters = getInitialCenter();

        for (int i=0; i<repeat; i++) {
            for (Cluster cluster : this.clusters) {
                cluster.clearElements();
            }

            for (StandardEntity e : this.entities) assignEntity(this.clusters, e);

            for (Cluster cluster : clusters) {
                cluster.updateCenter();
            }
        }
    }

    private ArrayList<Cluster> getInitialCenter() {
        ArrayList<Cluster> result = new ArrayList<>();
        ArrayList<StandardEntity> independents = new ArrayList<>(this.entities);
        for (int i = 0; i< clusterSize; i++) {
            result.add(new Cluster());
        }

        Random random = new Random(SEED);

        int rand = random.nextInt(independents.size());
        result.get(0).addElements(independents.get(rand));
        result.get(0).updateCenter();



        for (int i = 0; i< clusterSize - 1; i++) {
            ArrayList<Double> probabilities = new ArrayList<>();
            double sumd = 0.0;

            for (StandardEntity e : independents) {
                double cx = result.get(i).getCenterX();
                double cy = result.get(i).getCenterY();
                double x = getEntityX(e);
                double y = getEntityY(e);

                double d = Math.hypot(x - cx, y - cy);
                probabilities.add(d);
                sumd += d;
            }

            for(int j=0; j<independents.size(); ++j) {
                probabilities.set(j,probabilities.get(j)/sumd);
            }

            double p = random.nextDouble();

            for (int j = 0; j < independents.size(); j++) {
                p -= probabilities.get(j);
                if (p <= 0) {
                    result.get(i+1).addElements(independents.get(j));
                    result.get(i+1).updateCenter();
                    independents.remove(j);
                    break;
                }
            }

        }

        return result;
    }

    private void assignEntity(ArrayList<Cluster> clusters, StandardEntity e) {
        double x = getEntityX(e);
        double y = getEntityY(e);

        Cluster result = null;
        double mind = Double.MAX_VALUE;
        for (Cluster c : clusters) {
            if (result == null) {
                result = c;
            }

            double cx = c.getCenterX();
            double cy = c.getCenterY();

            double d = Math.hypot(cx - x, cy - y);

            if(d < mind) {
                mind = d;
                result = c;
            }
        }

        if (result != null) {
            result.addElements(e);
        }
    }

    private void assignAgents(List<StandardEntity> agentList) {
        int clusterIndex = 0;
        agentList.sort(new IdSorter());
        while (!agentList.isEmpty()) {
            double centerX = this.getClusterCenterX(clusterIndex);
            double centerY = this.getClusterCenterY(clusterIndex);

            StandardEntity nearAgent = null;
            double mind = Double.MAX_VALUE;
            for (StandardEntity agent : agentList) {
                if (nearAgent == null) {
                    nearAgent = agent;
                } else {
                    Human human = (Human) agent;
                    final double x = human.getX();
                    final double y = human.getY();

                    final double d = Math.hypot(x - centerX, y - centerY);

                    if (d < mind){
                        mind = d;
                        nearAgent = agent;
                    }
                }
            }

            if (nearAgent != null) {
                this.assigns.put(nearAgent.getID(), clusterIndex);
            }
            agentList.remove(nearAgent);
            clusterIndex++;
            if (clusterIndex >= this.getClusterNumber()) {
                clusterIndex = 0;
            }
        }
    }

    public static double getEntityX(StandardEntity e) {
        int x = 0;
        if (e instanceof Human) {
            Human h = (Human)e;
            if (h.isXDefined()) x = h.getX();
        }
        else if (e instanceof Area) {
            Area a = (Area)e;
            if (a.isXDefined()) x = a.getX();
        }
        else if (e instanceof Blockade) {
            Blockade b = (Blockade)e;
            if (b.isXDefined()) x = b.getX();
        }
        return x;
    }

    public static double getEntityY(StandardEntity e) {
        int y = 0;
        if (e instanceof Human) {
            Human h = (Human)e;
            if (h.isYDefined()) y = h.getY();
        }
        else
        if (e instanceof Area) {
            Area a = (Area)e;
            if (a.isYDefined()) y = a.getY();
        }
        else
        if (e instanceof Blockade) {
            Blockade b = (Blockade)e;
            if (b.isYDefined()) y = b.getY();
        }
        return y;
    }

    private static class Cluster {
        private double centerX;
        private double centerY;

        private List<StandardEntity> elements;

        public Cluster() {
            this.elements = new LinkedList<>();
            this.centerX = 0;
            this.centerY = 0;
        }

        public Cluster(double x, double y, List<StandardEntity> elements) {
            this.centerX = x;
            this.centerY = y;
            this.elements = elements;
        }

        public void addElements(StandardEntity e) {
            this.elements.add(e);
        }

        public List<StandardEntity> getElements() {
            return this.elements;
        }

        public void clearElements() {
            this.elements.clear();
        }

        public void updateCenter() {
            if (this.elements.isEmpty()){
                return;
            }

            double sumx = 0;
            double sumy = 0;

            for (StandardEntity e : this.elements) {
                sumx += getEntityX(e);
                sumy += getEntityY(e);
            }

            this.centerX = sumx / this.elements.size();
            this.centerY = sumy / this.elements.size();
        }

        public double getCenterX() {
            return this.centerX;
        }

        public double getCenterY() {
            return this.centerY;
        }
    }

    private static class IdSorter implements Comparator<StandardEntity> {
        public int compare(StandardEntity a, StandardEntity b) {
            int d1 = a.getID().getValue();
            int d2 = b.getID().getValue();
            return d1 - d2;
        }
    }
}
